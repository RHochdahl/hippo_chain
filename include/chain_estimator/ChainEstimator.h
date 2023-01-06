#ifndef CHAINESTIMATOR_H
#define CHAINESTIMATOR_H


#include <utility>
#include <memory>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <hippo_chain/AddVehicles.h>
#include <hippo_chain/ChainState.h>

#include <hippo_chain/include/utils/ConfigProvider.h>
#include <hippo_chain/include/chain_estimator/vehicle_estimator/VehicleEstimator.h>
#include <hippo_chain/include/chain_estimator/vehicle_estimator/BaseVehicleEstimator.h>
#include <hippo_chain/include/chain_estimator/vehicle_estimator/ChildVehicleEstimator.h>
#include <hippo_chain/include/chain_estimator/vehicle_estimator/ChildFactory.hpp>


class ChainEstimator
{
private:
    std::shared_ptr<ros::NodeHandle> nh;
    ConfigProvider configProvider;
    ros::Publisher statePub;
    ros::ServiceServer addVehiclesSrv;
    ros::ServiceClient stopVehiclesSrv;
    ros::Rate rate;

    std::shared_ptr<std::map<int, int>> idMap;

    std::vector<std::shared_ptr<VehicleEstimator>> vehicleEstimators;

    struct VehicleParam
    {
        std::string name;
        int publicId;
        int parentId;
        std::string jointType;
    };

    int numVehicles;


    void addVehicle(std::shared_ptr<VehicleEstimator> vehicle, const int publicId)
    {
        if (!idMap->insert(std::make_pair(publicId, numVehicles)).second) throw addition_error("Vehicle has already been added.");
        vehicleEstimators.push_back(vehicle);
        numVehicles++;

        ROS_INFO("Added vehicle with ID %i to chain estimator.", publicId);
    }

    void addChildVehicle(const VehicleParam param)
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

    void addBaseVehicle(const VehicleParam param)
    {
        if (vehicleEstimators.size())   throw addition_error("Base vehicle already exists!");
        if (idMap->size())              throw addition_error("Base vehicle already exists!");
        if (param.publicId < 0)         ROS_FATAL("Base vehicle has invalid ID!");
        if (param.parentId != -1)       ROS_FATAL("Base vehicle has incorrect parent ID!");

        addVehicle(std::make_shared<BaseVehicleEstimator>(param.name), param.publicId);
    }

    void readVehicleParams(const std::string& name, VehicleParam& param) const
    {
        param.name = name;
        param.publicId = shared::getID(param.name);
        if (param.publicId < 0) ROS_FATAL("ID could not be read from '%s'!", param.name.c_str());
        std::string parentName;
        if (!configProvider.getValue(param.name + "/parent", parentName)) ROS_FATAL("Vehicle '%s' does not name a parent!", param.name.c_str());
        if (parentName == "base") {
            param.parentId = -1;
        } else {
            param.parentId = shared::getID(parentName);
            if (param.parentId < 0) ROS_FATAL("ID could not be read from '%s'!", parentName.c_str());
            if (!configProvider.getValue(param.name + "/joint/type", param.jointType)) ROS_FATAL("Vehicle '%s' does not specify 'joint/type'!", param.name.c_str());
        }
    }

    bool addVehicles(const std::vector<std::string> vehicles)
    {
        std::vector<VehicleParam> waitingList(0);

        for (auto it=vehicles.begin(); it!=vehicles.end(); it++) {
            VehicleParam param;
            readVehicleParams(*it, param);
            waitingList.push_back(param);
        }

        {
            auto it=waitingList.begin();
            while(it!=waitingList.end()) {
                if (it->parentId == -1) {
                    try
                    {
                        addBaseVehicle(*it);
                    }
                    catch(const std::exception& e) {}
                    it = waitingList.erase(it);
                } else it++;
            }
        }
        if (!numVehicles) ROS_FATAL("No base vehicle could be constructed!");
        {
            auto it=waitingList.begin();
            while(it!=waitingList.end()) {
                if (idMap->count(it->parentId)) {
                    try
                    {
                        addChildVehicle(*it);
                    }
                    catch(const std::exception& e) {}
                    waitingList.erase(it);
                    it=waitingList.begin();
                } else it++;
            }
        }
        
        if (waitingList.size()) {
            ROS_ERROR("Some vehicles could not be added to chain estimator!");
            return false;
        }
        return true;
    }

    bool addVehiclesCallback(hippo_chain::AddVehicles::Request& request, hippo_chain::AddVehicles::Response& response)
    {
        return addVehicles(request.vehicle_names);
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
    : ChainEstimator(rate)
    {
        if (!vehicles.size()) {
            vehicles = configProvider.getChainVehicleNames();
            if (!vehicles.size()) ROS_FATAL("No vehicle names were given!");
        }
        
        addVehicles(vehicles);

        ROS_INFO("Constructed chain estimator for %i vehicles", numVehicles);
    }

    ChainEstimator(const double rate)
    : nh(new ros::NodeHandle())
    , configProvider(nh)
    , statePub(nh->advertise<hippo_chain::ChainState>("chain/state", 1))
    , addVehiclesSrv(nh->advertiseService("chain_estimator/addVehicles", &ChainEstimator::addVehiclesCallback, this))
    , stopVehiclesSrv(nh->serviceClient<std_srvs::Empty>("chain_controller/pause"))
    , rate(rate)
    , idMap(new std::map<int, int>())
    , vehicleEstimators(0)
    , numVehicles(0)
    {}

    ChainEstimator() : ChainEstimator(20.0) {}

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