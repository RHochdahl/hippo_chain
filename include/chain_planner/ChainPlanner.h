#ifndef CHAINPLANNER_H
#define CHAINPLANNER_H


#include <hippo_chain/include/common/ChainNodeTemplate.h>

#include <dynamic_reconfigure/server.h>
#include <hippo_chain/ChainTargetConfig.h>
#include <hippo_chain/ChainState.h>


class ChainPlanner : public ChainNodeTemplate
{
private:
    ros::Publisher targetPub;

    std::vector<int> jointPosSignList;

    dynamic_reconfigure::Server<hippo_chain::ChainTargetConfig> server;
    dynamic_reconfigure::Server<hippo_chain::ChainTargetConfig>::CallbackType f;

    hippo_chain::ChainState msg;


    void addVehicle(const VehicleParam& param)
    {
        if (!idMap->insert(std::make_pair(param.publicId, numVehicles)).second) throw addition_error("Vehicle has already been added.");
        dofList.push_back(param.dof);
        jointPosSignList.push_back(shared::sgn(param.jointPos));
        numVehicles++;
    }

    void addChildVehicle(const VehicleParam& param)
    {
        if (param.publicId < 0)             throw addition_error("Vehicle has invalid ID! Couldn't add vehicle.");
        if (param.parentId < 0)             throw addition_error("Named parent vehicle has invalid ID! Couldn't add vehicle.");
        if (!idMap->count(param.parentId))  throw addition_error("Parent has not been added yet.");
        if (idMap->count(param.publicId))   throw addition_error("Vehicle has already been added.");

        addVehicle(param);
    }

    void addBaseVehicle(const VehicleParam& param)
    {
        if (numVehicles)                throw addition_error("Base vehicle already exists!");
        if (idMap->size())              throw addition_error("Base vehicle already exists!");
        if (param.publicId < 0)         ROS_FATAL("Base vehicle has invalid ID!");
        if (param.parentId != -1)       ROS_FATAL("Base vehicle has incorrect parent ID!");

        addVehicle(param);
    }

    void reconfigureCallback(hippo_chain::ChainTargetConfig &config, uint32_t level)
    {
        if (!config.update) return;
        msg.data.clear();

        // base
        hippo_chain::ChainVehicleState baseState;
        baseState.vehicle_id = idMap->at(0);
        double halfYaw = config.yaw / 2;
        baseState.pose = {config.x, config.y, config.z, std::cos(halfYaw), 0, 0, std::sin(halfYaw)};
        baseState.twist = std::vector<double>(6, 0);
        msg.data.push_back(baseState);

        for (int i=1; i<numVehicles; i++) {
            hippo_chain::ChainVehicleState childState;
            childState.vehicle_id = idMap->at(i);
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
    : ChainNodeTemplate("chain_planner", vehicles, 2.0)
    , targetPub(nh->advertise<hippo_chain::ChainState>("chain/target", 1))
    , jointPosSignList()
    , server(ros::NodeHandle("ChainPlanner"))
    , f(boost::bind(&ChainPlanner::reconfigureCallback, this, _1, _2))
    {
        addVehicles(vehicles);
        server.setCallback(f);
        ROS_INFO("Constructed chain planner for %i vehicles", numVehicles);
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