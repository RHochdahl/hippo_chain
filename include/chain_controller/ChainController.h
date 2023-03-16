#ifndef CHAINCONTROLLER_H
#define CHAINCONTROLLER_H


#include <hippo_chain/include/common/ChainNodeTemplate.h>

#include <ctime>

#include <hippo_chain/ChainControllerConfig.h>

#include <hippo_chain/include/chain_controller/input_provider/InputProvider.h>
#include <hippo_chain/include/chain_controller/least_squares_solver/LeastSquaresSolver.h>
#include <hippo_chain/include/chain_controller/least_squares_solver/LQPSolver.h>
#include <hippo_chain/include/chain_controller/least_squares_solver/MatlabLSQSolver.h>
#include <hippo_chain/include/chain_controller/least_squares_solver/MatlabLSQSolver2.h>
#include <hippo_chain/include/chain_controller/vehicle_controller/VehicleController.h>
#include <hippo_chain/include/chain_controller/vehicle_controller/BaseVehicleController.h>
#include <hippo_chain/include/chain_controller/vehicle_controller/ChildVehicleController.h>
#include <hippo_chain/include/chain_controller/vehicle_controller/ChildFactory.hpp>


class ChainController : public ChainNodeTemplate
{
private:
    bool running;
    bool modified;

    ros::ServiceServer startSrv;
    ros::ServiceServer pauseSrv;
    ros::ServiceServer fixBaseSrv;

    std::vector<std::shared_ptr<VehicleController>> vehicleControllers;
    std::unique_ptr<InputProvider> inputProvider;

    int totalDof;

    std::vector<int> startIdxList;

    Eigen::MatrixXd B;
    Eigen::VectorXd eta;
    Eigen::VectorXd nu;

    std::unique_ptr<LeastSquaresSolver> lsqSolver;

    dynamic_reconfigure::Server<hippo_chain::ChainControllerConfig> server;
    dynamic_reconfigure::Server<hippo_chain::ChainControllerConfig>::CallbackType f;


    void addVehicle(std::shared_ptr<VehicleController> vehicle, const int publicId)
    {
        if (!idMap->insert(std::make_pair(publicId, numVehicles)).second) throw addition_error("Vehicle has already been added.");
        idList->push_back(publicId);
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

        modified = true;
    }

    void addChildVehicle(const VehicleParam& param)
    {
        if (running)                        throw addition_error("Controller still running. Couldn't add vehicle.");
        if (param.publicId < 0)             throw addition_error("Vehicle has invalid ID! Couldn't add vehicle.");
        if (param.parentId < 0)             throw addition_error("Named parent vehicle has invalid ID! Couldn't add vehicle.");
        if (!idMap->count(param.parentId))  throw addition_error("Parent has not been added yet.");
        if (idMap->count(param.publicId))   throw addition_error("Vehicle has already been added.");

        try
        {
            addVehicle(ChildFactory::bearChild(vehicleControllers[idMap->at(param.parentId)], param.name, numVehicles, param.jointType), param.publicId);
        }
        catch(const std::invalid_argument& e)
        {
            throw addition_error(e.what());
        }
    }

    void addBaseVehicle(const VehicleParam& param)
    {
        if (vehicleControllers.size())  throw addition_error("Base vehicle already exists!");
        if (idMap->size())              throw addition_error("Base vehicle already exists!");
        if (param.publicId < 0)         ROS_FATAL("Base vehicle has invalid ID!");
        if (param.parentId != -1)       ROS_FATAL("Base vehicle has incorrect parent ID!");

        addVehicle(std::make_shared<BaseVehicleController>(param.name), param.publicId);
    }

    void sendThrusterCommands(const Eigen::Ref<const Eigen::VectorXd>& thrusterOutputs)
    {
        assert(4*numVehicles == thrusterOutputs.rows());
        const double* data = thrusterOutputs.data();
        for (auto it=vehicleControllers.begin(); it!=vehicleControllers.end(); it++, data+=4) {
            (*it)->setThrusters(data);
        }
    }

    void stopThrusters()
    {
        for (auto it=vehicleControllers.begin(); it!=vehicleControllers.end(); it++) {
            (*it)->stopThrusters();
        }
    }

    void pause()
    {
        if (!running) return;
        stopThrusters();
        running = false;
        ROS_WARN("Chain Controller paused!");
    }

    void start(const ros::Duration timeOut = ros::Duration(5.0))
    {
        ros::Time startTime = ros::Time::now();
        if (modified) {
            lsqSolver.reset(new MatlabLSQSolver(4*numVehicles));
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

    bool fixBaseCallback(hippo_chain::FixBase::Request& req, hippo_chain::FixBase::Response& res)
    {
        if (!numVehicles) return false;

        reinterpret_cast<BaseVehicleController*>(vehicleControllers.front().get())->fix(req.fix);
        return true;
    }

    void reconfigureCallback(hippo_chain::ChainControllerConfig &config, uint32_t level)
    {
        if (config.run && !running) start();
        else if (!config.run && running) pause();
        config.run = running;

        if (numVehicles) {
            if (level) config.fixBase = reinterpret_cast<BaseVehicleController*>(vehicleControllers.front().get())->isFixed();
            else config.fixBase = reinterpret_cast<BaseVehicleController*>(vehicleControllers.front().get())->fix(config.fixBase);
        }
    }
    

public:
    ChainController(std::vector<std::string> vehicles, const bool autoStart, const double rate)
    : ChainNodeTemplate("chain_controller", vehicles, rate)
    , running(false)
    , modified(true)
    , startSrv(nh->advertiseService("chain_controller/start", &ChainController::startCallback, this))
    , pauseSrv(nh->advertiseService("chain_controller/pause", &ChainController::pauseCallback, this))
    , fixBaseSrv(nh->advertiseService("chain_controller/fixBase", &ChainController::fixBaseCallback, this))
    , vehicleControllers(0)
    , totalDof(0)
    , startIdxList(0)
    , server(ros::NodeHandle("ChainController"))
    , f(boost::bind(&ChainController::reconfigureCallback, this, _1, _2))
    {
        addVehicles(vehicles);

        lsqSolver.reset(new MatlabLSQSolver(4*numVehicles));
        inputProvider.reset(new InputProvider(idMap));

        modified = false;

        nu = Eigen::VectorXd::Zero(4*numVehicles);
        B = Eigen::MatrixXd::Zero(totalDof, 4*numVehicles);
        eta = Eigen::VectorXd::Zero(totalDof);

        server.setCallback(f);

        ROS_INFO("Constructed chain controller for %i vehicles", numVehicles);

        if (autoStart) {
            ros::Duration(5.0).sleep();
            start();
        }
    }

    ChainController()
    : ChainController(std::vector<std::string>(), false, std::numeric_limits<double>::quiet_NaN())
    {}

    ~ChainController()
    {
        if (ros::ok()) {
            stopThrusters();
        }
    }

    void spin()
    {
        while (ros::ok()) {
//            const clock_t begin_time = clock();
            ros::spinOnce();
//            std::cout << "Spin: " << double(clock() - begin_time) * (1000.0/CLOCKS_PER_SEC) << " ms\n";

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
                auto it=vehicleControllers.begin();
                for (; it!=vehicleControllers.end(); it++, id++) {
                    (*it)->updateVehicleState(inputProvider->getState(id));
                    (*it)->calcDecoupledWrenches(inputProvider->getTarget(id));

                    (*it)->calcB(B.middleCols<4>(4*id), startIdxList);
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
                auto vehiclesIt = vehicleControllers.rbegin();
                for (; vehiclesIt!=vehicleControllers.rend(); idxIt++, vehiclesIt++) {
                    (*vehiclesIt)->calcEta(eta, *idxIt);
                }
            }

//            std::cout << "Controller: " << double(clock() - begin_time) * (1000.0/CLOCKS_PER_SEC) << " ms\n";

            lsqSolver->solve(B, eta, nu);
            sendThrusterCommands(nu);

//            std::cout << "All: " << double(clock() - begin_time) * (1000.0/CLOCKS_PER_SEC) << " ms\n";
            rate.sleep();
        }
    }
};


#endif // CHAINCONTROLLER_H