#ifndef CHAINWATCHER_H
#define CHAINWATCHER_H


#include <hippo_chain/include/common/ChainNodeTemplate.h>

#include <hippo_chain/include/chain_watcher/Boundaries.h>
#include <hippo_chain/include/chain_watcher/VehicleWatcher.h>


class ChainWatcher : public ChainNodeTemplate
{
private:
    ros::ServiceClient stopVehiclesSrv;

    std::shared_ptr<Boundaries const> bounds;

    std::vector<std::unique_ptr<VehicleWatcher>> watchers;


    void addVehicle(const VehicleParam& param)
    {
        if (!idMap->insert(std::make_pair(param.publicId, numVehicles)).second) throw addition_error("Vehicle has already been added.");

        watchers.push_back(std::move(std::unique_ptr<VehicleWatcher>(new VehicleWatcher(param.name, bounds))));

        numVehicles++;
    }

    void addChildVehicle(const VehicleParam& param)
    {
        addVehicle(param);
    }

    void addBaseVehicle(const VehicleParam& param) 
    {
        addVehicle(param);
    }

    void stopVehicles()
    {
        std_srvs::Empty srv;
        if (!stopVehiclesSrv.call(srv)) {
            ROS_FATAL("Watcher failed to stop vehicles!");
        }
    }


public:
    ChainWatcher(std::vector<std::string> vehicles)
    : ChainNodeTemplate("chain_watcher", vehicles, 10.0)
    , stopVehiclesSrv(nh->serviceClient<std_srvs::Empty>("chain_controller/pause"))
    , bounds()
    , watchers()
    {
        Boundaries initBounds;
        if (!configProvider.getValue("bounds/x/lower", initBounds.x.lower) ||
            !configProvider.getValue("bounds/x/upper", initBounds.x.upper) ||
            !configProvider.getValue("bounds/y/lower", initBounds.y.lower) ||
            !configProvider.getValue("bounds/y/upper", initBounds.y.upper) ||
            !configProvider.getValue("bounds/z/lower", initBounds.z.lower) ||
            !configProvider.getValue("bounds/z/upper", initBounds.z.upper))
            ROS_FATAL("Could not read boundaries!");

        bounds.reset<Boundaries const>(const_cast<Boundaries* const>(new Boundaries(initBounds)));

        addVehicles(vehicles);
        ROS_INFO("Constructed chain watcher for %i vehicles", numVehicles);
    }

    ChainWatcher() : ChainWatcher(std::vector<std::string>()) {}

    ~ChainWatcher()
    {
        if (ros::ok()) stopVehicles();
    }


    void spin()
    {
        while(ros::ok()) {
            try
            {
                ros::spinOnce();
            }
            catch(const std::exception& e)
            {
                stopVehicles();
            }
            
            rate.sleep();
        }
    }
};


#endif  // CHAINWATCHER_H