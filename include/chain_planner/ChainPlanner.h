#ifndef CHAINPLANNER_H
#define CHAINPLANNER_H


#include <utility>
#include <memory>
#include <ros/ros.h>
#include <hippo_chain/AddVehicles.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <hippo_chain/ChainTargetConfig.h>
#include <hippo_chain/ChainState.h>

#include <hippo_chain/include/common/ConfigProvider.h>
#include <hippo_chain/include/common/sharedAlgorithms.hpp>


class ChainPlanner
{
private:
    std::shared_ptr<ros::NodeHandle> nh;
    ConfigProvider configProvider;
    ros::Publisher targetPub;
    ros::ServiceServer addVehiclesSrv;
    ros::Rate rate;

    int numVehicles;
    std::map<int, int> idMap;
    std::vector<int> dofList;
    std::vector<int> jointPosSignList;

    dynamic_reconfigure::Server<hippo_chain::ChainTargetConfig> server;
    dynamic_reconfigure::Server<hippo_chain::ChainTargetConfig>::CallbackType f;

    hippo_chain::ChainState msg;

    struct VehicleParam
    {
        std::string name;
        int publicId;
        int parentId;
        int dof;
        double distToJoint;
    };


    void addVehicle(const VehicleParam& param)
    {
        if (!idMap.insert(std::make_pair(param.publicId, numVehicles)).second) throw addition_error("Vehicle has already been added.");
        dofList.push_back(param.dof);
        jointPosSignList.push_back(shared::sgn(param.distToJoint));
        numVehicles++;
    }

    void addChildVehicle(const VehicleParam& param)
    {
        if (param.publicId < 0)             throw addition_error("Vehicle has invalid ID! Couldn't add vehicle.");
        if (param.parentId < 0)             throw addition_error("Named parent vehicle has invalid ID! Couldn't add vehicle.");
        if (!idMap.count(param.parentId))   throw addition_error("Parent has not been added yet.");
        if (idMap.count(param.publicId))    throw addition_error("Vehicle has already been added.");

        addVehicle(param);
    }

    void addBaseVehicle(const VehicleParam& param)
    {
        if (numVehicles)                throw addition_error("Base vehicle already exists!");
        if (idMap.size())               throw addition_error("Base vehicle already exists!");
        if (param.publicId < 0)         ROS_FATAL("Base vehicle has invalid ID!");
        if (param.parentId != -1)       ROS_FATAL("Base vehicle has incorrect parent ID!");

        addVehicle(param);
    }

    bool addVehicles(const std::vector<std::string>& vehicles)
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
                        addVehicle(*it);
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
                if (idMap.count(it->parentId)) {
                    try
                    {
                        addVehicle(*it);
                    }
                    catch(const std::exception& e) {}
                    waitingList.erase(it);
                    it=waitingList.begin();
                } else it++;
            }
        }
        
        if (waitingList.size()) {
            ROS_ERROR("Some vehicles could not be added to chain planner!");
            return false;
        }
        return true;
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
            param.dof = 6;
            param.distToJoint = 0.0;
        } else {
            param.parentId = shared::getID(parentName);
            if (param.parentId < 0) ROS_FATAL("ID could not be read from '%s'!", parentName.c_str());
            if (!configProvider.getValue(param.name + "/joint/dof", param.dof)) ROS_FATAL("Vehicle '%s' does not specify 'joint/dof'!", param.name.c_str());
            if (!configProvider.getValue(param.name + "/joint/x_pos", param.distToJoint)) ROS_FATAL("Vehicle '%s' does not specify 'joint/x_pos'!", param.name.c_str());
        }
    }


    bool addVehiclesCallback(hippo_chain::AddVehicles::Request& request, hippo_chain::AddVehicles::Response& response)
    {
        return addVehicles(request.vehicle_names);
    }

    void reconfigureCallback(hippo_chain::ChainTargetConfig &config, uint32_t level)
    {
        if (!config.update) return;
        msg.data.clear();

        // base
        hippo_chain::ChainVehicleState baseState;
        baseState.vehicle_id = idMap.at(0);
        double halfYaw = config.yaw / 2;
        baseState.pose = {config.x, config.y, config.z, std::cos(halfYaw), 0, 0, std::sin(halfYaw)};
        baseState.twist = std::vector<double>(6, 0);
        msg.data.push_back(baseState);

        for (int i=1; i<numVehicles; i++) {
            hippo_chain::ChainVehicleState childState;
            childState.vehicle_id = idMap.at(i);
            childState.pose = std::vector<double>(dofList[i]);
            childState.pose.front() = -config.form * jointPosSignList[i] * 2 * M_PI / std::max(numVehicles, 3);
            for (auto it=childState.pose.begin()+1, end=childState.pose.end(); it<end; it++) *it = 0;
            childState.twist = std::vector<double>(dofList[i], 0);
            msg.data.push_back(childState);
        }
        config.update = false;
    }

    void publish()
    {
        msg.header.stamp = ros::Time::now();
        targetPub.publish(msg);
    }
    

public:
    ChainPlanner(std::vector<std::string> vehicles)
    : nh(new ros::NodeHandle())
    , configProvider(nh)
    , targetPub(nh->advertise<hippo_chain::ChainState>("chain/target", 1))
    , addVehiclesSrv(nh->advertiseService("chain_planner/addVehicles", &ChainPlanner::addVehiclesCallback, this))
    , rate(ros::Duration(0.5))
    , numVehicles(0)
    , idMap()
    , dofList()
    , jointPosSignList()
    , server(ros::NodeHandle("ChainPlanner"))
    , f(boost::bind(&ChainPlanner::reconfigureCallback, this, _1, _2))
    {
        if (!vehicles.size()) {
            vehicles = configProvider.getChainVehicleNames();
            if (!vehicles.size()) ROS_FATAL("No vehicle names were given!");
        }
        
        addVehicles(vehicles);

        ROS_INFO("Constructed chain planner for %i vehicles", numVehicles);

        server.setCallback(f);
    }
    
    ChainPlanner() : ChainPlanner(std::vector<std::string>()) {}

    void spin()
    {
        while (ros::ok()) {
            ros::spinOnce();

            if (!msg.data.size()) continue;

            publish();

            rate.sleep();
        }
    }
};


#endif // CHAINPLANNER_H