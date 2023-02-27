#ifndef CHAINESTIMATOR_H
#define CHAINESTIMATOR_H


#include <hippo_chain/include/common/ChainNodeTemplate.h>

#include <hippo_chain/ChainState.h>

#include <hippo_chain/include/chain_estimator/vehicle_estimator/VehicleEstimator.h>
#include <hippo_chain/include/chain_estimator/vehicle_estimator/BaseVehicleEstimator.h>
#include <hippo_chain/include/chain_estimator/vehicle_estimator/ChildVehicleEstimator.h>
#include <hippo_chain/include/chain_estimator/vehicle_estimator/ChildFactory.hpp>


class ChainEstimator : public ChainNodeTemplate
{
private:
    ros::Publisher statePub;
    ros::ServiceClient stopVehiclesSrv;

    std::vector<std::shared_ptr<VehicleEstimator>> vehicleEstimators;


    void addVehicle(std::shared_ptr<VehicleEstimator> vehicle, const int publicId)
    {
        if (!idMap->insert(std::make_pair(publicId, numVehicles)).second) throw addition_error("Vehicle has already been added.");
        idList->push_back(publicId);
        vehicleEstimators.push_back(vehicle);
        numVehicles++;

        ROS_INFO("Added vehicle with ID %i to chain estimator.", publicId);
    }

    void addChildVehicle(const VehicleParam& param)
    {
        if (param.publicId < 0)             throw addition_error("Vehicle has invalid ID! Couldn't add vehicle.");
        if (param.parentId < 0)             throw addition_error("Named parent vehicle has invalid ID! Couldn't add vehicle.");
        if (!idMap->count(param.parentId))  throw addition_error("Parent has not been added yet.");
        if (idMap->count(param.publicId))   throw addition_error("Vehicle has already been added.");

        try
        {
            addVehicle(ChildFactory::bearChild(vehicleEstimators[idMap->at(param.parentId)], param.name, param.jointType), param.publicId);
        }
        catch(const std::invalid_argument& e)
        {
            throw addition_error(e.what());
        }
    }

    void addBaseVehicle(const VehicleParam& param)
    {
        if (vehicleEstimators.size())   throw addition_error("Base vehicle already exists!");
        if (idMap->size())              throw addition_error("Base vehicle already exists!");
        if (param.publicId < 0)         ROS_FATAL("Base vehicle has invalid ID!");
        if (param.parentId != -1)       ROS_FATAL("Base vehicle has incorrect parent ID!");

        addVehicle(std::make_shared<BaseVehicleEstimator>(param.name), param.publicId);
    }

    void stopVehicles()
    {
        std_srvs::Empty srv;
        if (!stopVehiclesSrv.call(srv)) {
            ROS_WARN("Estimator failed to stop vehicles!");
        }
    }


public:
    ChainEstimator(std::vector<std::string> vehicles, const double rate)
    : ChainNodeTemplate("chain_estimator", vehicles, rate)
    , statePub(nh->advertise<hippo_chain::ChainState>("chain/state", 1))
    , stopVehiclesSrv(nh->serviceClient<std_srvs::Empty>("chain_controller/pause"))
    , vehicleEstimators(0)
    {
        addVehicles(vehicles);
        ROS_INFO("Constructed chain estimator for %i vehicles", numVehicles);
    }

    ChainEstimator()
    : ChainEstimator(std::vector<std::string>(), std::numeric_limits<double>::quiet_NaN())
    {}

    ~ChainEstimator()
    {
        ROS_WARN("Chain Estimator terminated!");
        stopVehicles();
    }

    void spinOnce()
    {
        hippo_chain::ChainState msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        for (auto it=vehicleEstimators.begin(); it!=vehicleEstimators.end(); it++) {
            if ((*it)->isTimedOut()) {
                rate.sleep();
                return;
            }
            (*it)->estimate();
            msg.data.push_back((*it)->getStateMsg());
        }
        statePub.publish(msg);
    }

    void spin()
    {
        while (ros::ok()) {
            try
            {
                ros::spinOnce();
            }
            catch(const timeout_error& e)
            {
                stopVehicles();
                rate.sleep();
                continue;
            }

            spinOnce();

            rate.sleep();
        }
    }
};


#endif // CHAINESTIMATOR_H