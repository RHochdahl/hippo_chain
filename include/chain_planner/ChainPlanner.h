#ifndef CHAINPLANNER_H
#define CHAINPLANNER_H


#include <hippo_chain/include/common/ChainNodeTemplate.h>

#include <dynamic_reconfigure/server.h>
#include <hippo_chain/ChainTargetConfig.h>
#include <hippo_chain/ChainState.h>

#include <hippo_chain/FixBase.h>

#include "VisualPose.h"
#include "target_modes/TargetMode.h"
#include "target_modes/joint_modes/JointStepWaveMode.h"
#include "target_modes/joint_modes/JointSineWaveMode.h"
#include "target_modes/joint_modes/JointRampWaveMode.h"
#include "target_modes/joint_modes/JointSetpointMode.h"
#include "target_modes/triangle_modes/TriangleCircleMode.h"


class ChainPlanner : public ChainNodeTemplate
{
private:
    ros::Publisher targetPub;

    std::vector<int> jointPosSignList;
    double basePosOffset;

    dynamic_reconfigure::Server<hippo_chain::ChainTargetConfig> server;
    dynamic_reconfigure::Server<hippo_chain::ChainTargetConfig>::CallbackType f;

    hippo_chain::ChainState msg;
    std::vector<std::shared_ptr<VisualPose>> visuals;

    int mode;

    std::unique_ptr<TargetMode> targetMode;


#define NEUTRAL_POSE 0.99, 1.9, -0.59, -M_PI_2


    void addVehicle(const VehicleParam& param)
    {
        if (!idMap->insert(std::make_pair(param.publicId, numVehicles)).second) throw addition_error("Vehicle has already been added.");
        idList->push_back(param.publicId);
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
        basePosOffset = std::abs(param.jointPos)*std::tan(M_PI/6);
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

        if (config.resetMode) {
            mode = config.mode;
        }

        switch (mode)
        {
        case 0:     // manual mode
            setModeManual(config, config.resetMode);
            break;

        case 1:     // joint angle
            ROS_INFO("Planner Mode: 'JointAngle'");
            setModeJointAngle(config, config.resetMode);
            break;

        case 2:     // triangle
            ROS_INFO("Planner Mode: 'MoveTriangle'");
            setModeMoveTriangle(config, config.resetMode);
            break;

        default:
            ROS_ERROR("Unexpected mode chosen!");
            targetMode.reset(new TargetMode());
            break;
        }

        config.mode = mode;
        config.resetMode = false;
        config.update = false;
    }

    void setModeManual(hippo_chain::ChainTargetConfig &config, const bool reset)
    {
        if (reset){
            targetMode.reset(new TargetMode());
        }

        setBaseState(config.x, config.y, config.z, TO_RAD*config.yaw);
        setChildStates(TO_RAD*config.jointAngle);
    }

    void setModeJointAngle(hippo_chain::ChainTargetConfig &config, const bool reset)
    {
        if (reset){
            setBaseState(NEUTRAL_POSE);
            switch (config.jointMode)
            {
            case 0:
                targetMode.reset(new JointStepWaveMode(TO_RAD*config.Amplitude, config.Period, config.Duration, boost::bind(&ChainPlanner::setChildStates, this, _1)));
                break;

            case 1:
                targetMode.reset(new JointSineWaveMode(TO_RAD*config.Amplitude, config.Period, config.Duration, boost::bind(&ChainPlanner::setChildStates, this, _1, _2)));
                break;

            case 2:
                targetMode.reset(new JointRampWaveMode(TO_RAD*config.Amplitude, config.Period, config.Duration, boost::bind(&ChainPlanner::setChildStates, this, _1, _2)));
                break;

            case 3:
                targetMode.reset(new JointSetpointMode(TO_RAD*config.Amplitude, TO_RAD*config.StepSize, config.Period, config.Duration, boost::bind(&ChainPlanner::setChildStates, this, _1)));
                break;

            default:
                targetMode.reset(new TargetMode());
                mode = 0;
                ROS_WARN("Unknown joint mode selected!");
                return;
            }

            targetMode->start(1.0);
        }
    }

    void setModeMoveTriangle(hippo_chain::ChainTargetConfig &config, const bool reset)
    {
        if (reset){
            setChildStates(-2 * M_PI / std::max(numVehicles, 3));
            switch (config.triangleMode)
            {
            case 0:
                targetMode.reset(new TriangleCircleMode(config.Center_x, config.Center_y, config.Radius, -config.Depth, config.Period, config.Duration, boost::bind(&ChainPlanner::setBaseState, this, _1, _2, _3, _4, _5, _6, _7, _8)));
                break;
            
            default:
                targetMode.reset(new TargetMode());
                mode = 0;
                ROS_WARN("Unknown triangle mode selected!");
                return;
            }

            targetMode->start(1.0);
        }

/*
        setBaseState(config.x+std::sin(TO_RAD*config.yaw)*basePosOffset,
                     config.y-std::cos(TO_RAD*config.yaw)*basePosOffset,
                     config.z,
                     TO_RAD*config.yaw);
*/
    }

    void setBaseState(const double x, const double y, const double z, const double yaw)
    {
        setBaseState(x, y, z, yaw, 0.0, 0.0, 0.0, 0.0);
    }

    void setBaseState(const double x, const double y, const double z, const double yaw,
                      const double u, const double v, const double w, const double omega)
    {
        hippo_chain::ChainVehicleState baseState;
        baseState.vehicle_id = idList->at(0);
        const double halfYaw = yaw / 2;
        baseState.pose = {x, y, z, std::cos(halfYaw), 0, 0, std::sin(halfYaw)};
        baseState.twist = {u, v, w, 0.0, 0.0, omega};
        visuals.front()->set(baseState.pose);
        msg.data.front() = baseState;
        for (int i=1; i<numVehicles; i++) {
            visuals[i]->refresh();
        }
    }

    void setChildStates(const double angle)
    {
        setChildStates(angle, 0.0);
    }

    void setChildStates(const double angle, const double omega)
    {
        const double maxAngle = 2 * M_PI / std::max(numVehicles, 3);
        for (int i=1; i<numVehicles; i++) {
            hippo_chain::ChainVehicleState childState;
            childState.vehicle_id = idList->at(i);
            childState.pose = std::vector<double>(dofList[i]);
            childState.pose.back() = -std::max(std::min(angle, maxAngle), -maxAngle) * jointPosSignList[i];
            for (auto it=childState.pose.begin(), end=childState.pose.end(); it<end-1; it++) *it = 0;
            childState.twist = std::vector<double>(dofList[i]);
            childState.twist.back() = -omega * jointPosSignList[i];
            for (auto it=childState.twist.begin(), end=childState.twist.end(); it<end-1; it++) *it = 0;
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
    ChainPlanner(std::vector<std::string> vehicles, const double rate)
    : ChainNodeTemplate("chain_planner", vehicles, rate)
    , targetPub(nh->advertise<hippo_chain::ChainState>("chain/target", 1))
    , jointPosSignList()
    , basePosOffset()
    , server(ros::NodeHandle(*nh, "ChainPlanner"))
    , f(boost::bind(&ChainPlanner::reconfigureCallback, this, _1, _2))
    , msg()
    , visuals()
    , mode(0)
    , targetMode(new TargetMode())
    {
        addVehicles(vehicles);
        msg.data.resize(numVehicles);
        setBaseState(NEUTRAL_POSE);
        setChildStates(0.0);
        server.setCallback(f);
        ROS_INFO("Constructed chain planner for %i vehicles", numVehicles);
    }

    ChainPlanner() : ChainPlanner(std::vector<std::string>(), std::nan("")) {}

    void spin()
    {
        while (ros::ok()) {
            ros::spinOnce();

            if (!msg.data.size()) continue;

            if (targetMode->set()) {
                mode = 0;
                setBaseState(NEUTRAL_POSE);
                setChildStates(0.0);
            }
            publish();

            rate.sleep();
        }
    }
};


#endif // CHAINPLANNER_H