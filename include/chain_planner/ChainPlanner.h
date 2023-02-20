#ifndef CHAINPLANNER_H
#define CHAINPLANNER_H


#include <hippo_chain/include/common/ChainNodeTemplate.h>

#include <dynamic_reconfigure/server.h>
#include <hippo_chain/ChainTargetConfig.h>
#include <hippo_chain/ChainState.h>

#include <hippo_chain/FixBase.h>

#include "VisualPose.h"
#include "periodic_modes/PeriodicMode.h"
#include "periodic_modes/PeriodicReconfigurationMode.h"


class ChainPlanner : public ChainNodeTemplate
{
private:
    ros::Publisher targetPub;

    std::vector<int> jointPosSignList;

    dynamic_reconfigure::Server<hippo_chain::ChainTargetConfig> server;
    dynamic_reconfigure::Server<hippo_chain::ChainTargetConfig>::CallbackType f;

    hippo_chain::ChainState msg;
    std::vector<std::shared_ptr<VisualPose>> visuals;

    int mode;

    std::unique_ptr<PeriodicMode> periodicMode;


#define NEUTRAL_POSE 0.85, 1.7, -0.5, M_PI_2


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
        visuals.push_back(std::make_shared<VisualPose>(param.name, param.jointPos, visuals[idMap->at(param.parentId)]));
    }

    void addBaseVehicle(const VehicleParam& param)
    {
        if (numVehicles)                throw addition_error("Base vehicle already exists!");
        if (idMap->size())              throw addition_error("Base vehicle already exists!");
        if (param.publicId < 0)         ROS_FATAL("Base vehicle has invalid ID!");
        if (param.parentId != -1)       ROS_FATAL("Base vehicle has incorrect parent ID!");

        addVehicle(param);
        visuals.push_back(std::make_shared<VisualPose>(param.name));
    }

    void reconfigureCallback(hippo_chain::ChainTargetConfig &config, uint32_t level)
    {
        if (!config.update) return;

        bool reset = false;
        if (config.setMode) {
            if (config.mode != mode) reset = true;
            mode = config.mode;
        }

        switch (mode)
        {
        case 0:     // manual mode
            setModeManual(config, reset);
            break;

        case 1:     // joint angle
            ROS_INFO("Planner Mode: 'JointAngle'");
            setModeJointAngle(config, reset);
            break;

        case 2:     // triangle
            ROS_INFO("Planner Mode: 'MoveTriangle'");
            setModeMoveTriangle(config, reset);
            break;

        default:
            ROS_ERROR("Unexpected mode chosen!");
            periodicMode.release();
            break;
        }

        config.mode = mode;
        config.setMode = false;
        config.update = false;
    }

    void setModeManual(hippo_chain::ChainTargetConfig &config, const bool reset)
    {
        if (reset){
            periodicMode.reset();
        }

        setBaseState(config.x, config.y, config.z, config.yaw);
        setChildStates(config.form);
    }

    void setModeJointAngle(hippo_chain::ChainTargetConfig &config, const bool reset)
    {
        if (reset){
            setBaseState(NEUTRAL_POSE);
            periodicMode.reset(new PeriodicReconfigurationMode(config.Period, config.Duration, boost::bind(&ChainPlanner::setChildStates, this, _1)));
            periodicMode->start(5.0);
        }
    }

    void setModeMoveTriangle(hippo_chain::ChainTargetConfig &config, const bool reset)
    {
        if (reset){
            setChildStates(1.0);
            periodicMode.reset();
        }

        setBaseState(config.x, config.y, config.z, config.yaw);
    }

    void setBaseState(const double x, const double y, const double z, const double yaw)
    {
        hippo_chain::ChainVehicleState baseState;
        baseState.vehicle_id = idMap->at(0);
        const double halfYaw = yaw / 2;
        baseState.pose = {x, y, z, std::cos(halfYaw), 0, 0, std::sin(halfYaw)};
        baseState.twist = std::vector<double>(6, 0);
        visuals.front()->set(baseState.pose);
        msg.data.front() = baseState;
    }

    void setChildStates(const double form)
    {
        for (int i=1; i<numVehicles; i++) {
            hippo_chain::ChainVehicleState childState;
            childState.vehicle_id = idMap->at(i);
            childState.pose = std::vector<double>(dofList[i]);
            childState.pose.front() = -form * jointPosSignList[i] * 2 * M_PI / std::max(numVehicles, 3);
            for (auto it=childState.pose.begin()+1, end=childState.pose.end(); it<end; it++) *it = 0;
            childState.twist = std::vector<double>(dofList[i], 0);
            visuals[i]->set(childState.pose);
            msg.data[i] = childState;
        }
    }

    void publish()
    {
        assert(msg.data.size() == numVehicles);
        msg.header.stamp = ros::Time::now();
        targetPub.publish(msg);

        for (auto it=visuals.begin(); it!=visuals.end(); it++) {
            (*it)->publish();
        }
    }
    

public:
    ChainPlanner(std::vector<std::string> vehicles)
    : ChainNodeTemplate("chain_planner", vehicles, 2.0)
    , targetPub(nh->advertise<hippo_chain::ChainState>("chain/target", 1))
    , jointPosSignList()
    , server(ros::NodeHandle("ChainPlanner"))
    , f(boost::bind(&ChainPlanner::reconfigureCallback, this, _1, _2))
    , msg()
    , visuals()
    , mode(0)
    {
        addVehicles(vehicles);
        msg.data.resize(numVehicles);
        setBaseState(NEUTRAL_POSE);
        setChildStates(0.0);
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