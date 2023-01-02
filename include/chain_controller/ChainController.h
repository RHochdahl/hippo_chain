#ifndef CHAINCONTROLLER_H
#define CHAINCONTROLLER_H


#include <utility>
#include <memory>
#include <ros/ros.h>
#include <hippo_chain/AddVehicles.h>
#include <std_srvs/Empty.h>
#include <hippo_chain/ChainControllerConfig.h>

#include "input_provider/InputProvider.h"
#include "least_squares_solver/LeastSquaresSolver.h"
#include "least_squares_solver/LQPSolver.h"
#include "vehicle_controller/VehicleController.h"
#include "vehicle_controller/BaseVehicleController.h"
#include "vehicle_controller/ChildVehicleController.h"
#include "vehicle_controller/ChildFactory.hpp"
#include "../utils/ConfigProvider.h"


class ChainController
{
private:
    bool running;
    bool modified;
    std::shared_ptr<ros::NodeHandle> nh;
    ConfigProvider configProvider;
    ros::ServiceServer startSrv;
    ros::ServiceServer pauseSrv;
    ros::ServiceServer addVehiclesSrv;
    ros::Rate rate;

    std::shared_ptr<std::map<int, int>> idMap;

    std::vector<std::shared_ptr<VehicleController>> vehicleControllers;
    std::unique_ptr<InputProvider> inputProvider;

    struct VehicleParam
    {
        std::string name;
        int publicId;
        int parentId;
        std::string jointType;
    };

    int numVehicles;
    int totalDof;

    std::vector<int> dofList;
    std::vector<int> startIdxList;

    Eigen::MatrixXd B;
    Eigen::VectorXd eta;
    Eigen::VectorXd nu;

    std::unique_ptr<LeastSquaresSolver> lsqSolver;

    dynamic_reconfigure::Server<hippo_chain::ChainControllerConfig> server;
    dynamic_reconfigure::Server<hippo_chain::ChainControllerConfig>::CallbackType f;


    bool addVehicle(std::shared_ptr<VehicleController> vehicle, const int publicId)
    {
        if (!idMap->insert(std::make_pair(publicId, numVehicles)).second) throw addition_error("Vehicle has already been added.");
        vehicleControllers.push_back(vehicle);
        numVehicles++;
        int dof = vehicle->getDof();
        dofList.push_back(dof);
        startIdxList.push_back(totalDof);
        totalDof += dof;

        nu = Eigen::VectorXd::Zero(4*numVehicles);
        B = Eigen::MatrixXd::Zero(totalDof, 4*numVehicles);
        eta = Eigen::VectorXd::Zero(totalDof);

        ROS_INFO("Added vehicle with ID %i to chain controller.", publicId);

        return modified = true;
    }

    bool addChildVehicle(const VehicleParam param)
    {
        if (running)                        throw addition_error("Controller still running. Couldn't add vehicle.");
        if (param.publicId < 0)             throw addition_error("Vehicle has invalid ID! Couldn't add vehicle.");
        if (param.parentId < 0)             throw addition_error("Named parent vehicle has invalid ID! Couldn't add vehicle.");
        if (!idMap->count(param.parentId))  throw addition_error("Parent has not been added yet.");
        if (idMap->count(param.publicId))   throw addition_error("Vehicle has already been added.");

        try
        {
            return addVehicle(ChildFactory::bearChild(vehicleControllers[idMap->at(param.parentId)], param.name, numVehicles, param.jointType), param.publicId);
        }
        catch(const std::invalid_argument& e)
        {
            throw addition_error(e.what());
        }
    }

    bool addBaseVehicle(const VehicleParam param)
    {
        if (vehicleControllers.size())  throw addition_error("Base vehicle already exists!");
        if (idMap->size())              throw addition_error("Base vehicle already exists!");
        if (param.publicId < 0)         ROS_FATAL("Base vehicle has invalid ID!");
        if (param.parentId != -1)       ROS_FATAL("Base vehicle has incorrect parent ID!");

        return addVehicle(std::make_shared<BaseVehicleController>(param.name), param.publicId);
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
            ROS_ERROR("Some vehicles could not be added to chain controller!");
            return false;
        }
        return true;
    }

    void sendThrusterCommands(const Eigen::VectorXd& thrusterOutputs)
    {
        int idx = 0;
        for (auto it=vehicleControllers.begin(); it!=vehicleControllers.end(); it++, idx+=4) {
            (*it)->publish(thrusterOutputs.block(idx, 0, 4, 1));
        }
    }

    void stopThrusters()
    {
        for (auto it=vehicleControllers.begin(); it!=vehicleControllers.end(); it++) {
            (*it)->publish(Eigen::Vector4d::Zero());
        }
    }

    void pause()
    {
        stopThrusters();
        running = false;
        ROS_WARN("Chain Controller paused!");
    }

    void start(const ros::Duration timeOut = ros::Duration(5.0))
    {
        ros::Time startTime = ros::Time::now();
        if (modified) {
            lsqSolver.reset(new LQPSolver(4*numVehicles));
            inputProvider->reset();
            modified = false;
        }
        running = true;
        ROS_INFO("Start chain controller.");

        while (inputProvider->isUnsafe()) {
            stopThrusters();    // ensure that vehicles remain armed
            ros::spinOnce();
            if (!running) return;
            if ((ros::Time::now()-startTime) > timeOut) {
                ROS_WARN("No state inputs received! Pause controller.");
                running = false;
                return;
            }
            ROS_WARN_THROTTLE(5.0, "Waiting for state inputs.");
        }
    }

    bool addVehiclesCallback(hippo_chain::AddVehicles::Request& request, hippo_chain::AddVehicles::Response& response)
    {
        return addVehicles(request.vehicle_names);
    }

    bool pauseCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
    {
        pause();
        return !running;
    }

    bool startCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
    {
        start();
        return running;
    }

    void reconfigureCallback(hippo_chain::ChainControllerConfig &config, uint32_t level)
    {
        if (config.run && !running) start();
        else if (!config.run && running) pause();
        config.run = running;
    }
    

public:
    ChainController(std::vector<std::string> vehicles, const bool autoStart, const double rate)
    : ChainController(rate)
    {
        if (!vehicles.size()) {
            vehicles = configProvider.getChainVehicleNames();
            if (!vehicles.size()) ROS_FATAL("No vehicle names were given!");
        }
        
        addVehicles(vehicles);

        lsqSolver.reset(new LQPSolver(4*numVehicles));
        inputProvider.reset(new InputProvider(idMap));

        ROS_INFO("Constructed chain controller for %i vehicles", numVehicles);

        nu = Eigen::VectorXd::Zero(4*numVehicles);
        B = Eigen::MatrixXd::Zero(totalDof, 4*numVehicles);
        eta = Eigen::VectorXd::Zero(totalDof);

        if (autoStart) {
            ros::Duration(5.0).sleep();
            start();
        }
    }

    ChainController(const double rate)
    : running(false)
    , modified(true)
    , nh(new ros::NodeHandle())
    , configProvider(nh)
    , startSrv(nh->advertiseService("chain_controller/start", &ChainController::startCallback, this))
    , pauseSrv(nh->advertiseService("chain_controller/pause", &ChainController::pauseCallback, this))
    , addVehiclesSrv(nh->advertiseService("chain_controller/addVehicles", &ChainController::addVehiclesCallback, this))
    , rate(rate)
    , idMap(new std::map<int, int>())
    , vehicleControllers(0)
    , numVehicles(0)
    , totalDof(0)
    , dofList(0)
    , startIdxList(0)
    , server(ros::NodeHandle("ChainController"))
    , f(boost::bind(&ChainController::reconfigureCallback, this, _1, _2))
    {
        server.setCallback(f);
    }

    ChainController() : ChainController(20.0) {}

    ~ChainController()
    {
        if (ros::ok()) {
            stopThrusters();
        }
    }

    void spin()
    {
        while (ros::ok()) {
            ros::spinOnce();

            if (!running) {
                stopThrusters();    // continue to send zeros to remain armed
                rate.sleep();
                continue;
            }

            if (inputProvider->isUnsafe()) {
                pause();
                rate.sleep();
                continue;
            }

            if (!inputProvider->hasNewInputs()) {
                sendThrusterCommands(nu);
                rate.sleep();
                continue;
            }

            try
            {
                int id = 0;
                std::vector<std::pair<uint, Eigen::Matrix<double, Eigen::Dynamic, 4>>> Bpart;
                for (auto it=vehicleControllers.begin(); it!=vehicleControllers.end(); it++, id++) {
                    (*it)->updateVehicleState(inputProvider->getState(id));
                    (*it)->calcDecoupledWrenches(inputProvider->getTarget(id));

                    (*it)->calcB(Bpart);
                    for (auto it=Bpart.begin(); it!=Bpart.end(); it++) {
                        B.block(startIdxList[it->first], 4*id, dofList[it->first], 4) = it->second;
                    }
                    Bpart.clear();
                }
            }
            catch(const auto_print_error& e)
            {
                pause();
                rate.sleep();
                continue;
            }

            {
                auto idxIt = startIdxList.rbegin();
                auto dofIt = dofList.rbegin();
                auto vehiclesIt = vehicleControllers.rbegin();
                for (; vehiclesIt!=vehicleControllers.rend(); idxIt++, dofIt++, vehiclesIt++) {
                    eta.block(*idxIt, 0, *dofIt, 1) = (*vehiclesIt)->calcEta();
                }
            }

            nu = lsqSolver->solve(B, eta);
            sendThrusterCommands(nu);

            rate.sleep();
        }
    }
};


#endif // CHAINCONTROLLER_H