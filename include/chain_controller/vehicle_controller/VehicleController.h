#ifndef VEHICLECONTROLLER_H
#define VEHICLECONTROLLER_H


#include <ros/ros.h>
#include <math.h>
#include <Eigen/Dense>

#include <dynamic_reconfigure/server.h>
#include <hippo_chain/VehicleControllerConfig.h>

#include <mavros_msgs/AttitudeTarget.h>

#include <hippo_chain/include/chain_controller/vehicle_model/VehicleModel.h>
#include <hippo_chain/include/chain_controller/state/StateProvider.h>
#include <hippo_chain/include/chain_controller/thruster_model/ThrusterModel.h>
#include <hippo_chain/include/common/typedefs.h>
#include <hippo_chain/include/common/sharedAlgorithms.hpp>
#include <hippo_chain/include/common/DynamicReconfigureManager.h>
#include <hippo_chain/include/common/Debugger.h>


class VehicleController
{
public:
    const int ID;

protected:
    std::shared_ptr<ros::NodeHandle> nh;
    ros::Publisher pub;
    Debugger debugger;
    std::shared_ptr<ConfigProvider> configProvider;

    VehicleModel vehicleModel;
    ThrusterModel thrusterModel;

    Eigen::Vector6d tau;    // desired wrenches

    Eigen::Vector6d xiAbs;  // abs velocity

    typedef boost::function<void(const hippo_chain::VehicleControllerConfig &, uint32_t)> CallbackType;
    CallbackType f;

    static inline std::unique_ptr<DynamicReconfigureManager<hippo_chain::VehicleControllerConfig>> dynamicReconfigureManager;

    enum DynamicReconfigureLevels {
        BASE = 1,
        CHILD = 2
    };


    virtual void updateControlParameters(const hippo_chain::VehicleControllerConfig& config, uint32_t level) = 0;

    template<int Dim>
    static inline Eigen::Matrix<double,Dim,1> limitError(const Eigen::Matrix<double,Dim,1>& error, const double maxError)
    {
        static_assert(Dim > 0);
        Eigen::Matrix<double,Dim,1> result;
        auto errorIt=error.data();
        auto end=errorIt+Dim;
        auto resultIt=result.data();
        for (; errorIt<end; errorIt++, resultIt++) {
            *resultIt = limitError(*errorIt, maxError);
        }
        return result;
    }

    static inline double limitError(const double error, const double maxError)
    {
        return (std::abs(error) > maxError) ? shared::sgn(error) * maxError : error;
    }


public:
    VehicleController(const std::string& name, const int id)
    : ID(id)
    , nh(new ros::NodeHandle(name))
    , pub(nh->advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 1))
    , debugger(nh->getNamespace(), "debug_control")
    , configProvider(new ConfigProvider(nh))
    , vehicleModel(configProvider)
    , thrusterModel(configProvider)
    , f(boost::bind(&VehicleController::updateControlParameters, this, _1, _2))
    {
        if (!dynamicReconfigureManager) {
            std::string configFilePath = "";
            if (ros::param::get("dynamic_reconfigure_dir", configFilePath)) configFilePath += "VecicleControllerConfig.csv";
            dynamicReconfigureManager.reset(new DynamicReconfigureManager<hippo_chain::VehicleControllerConfig>("VehicleControllers", configFilePath));
        }
        dynamicReconfigureManager->registerCallback(f, this);
    }

    ~VehicleController()
    {
        if (ros::ok()) {
            stopThrusters();
        }
        dynamicReconfigureManager->removeCallback(this);
    }


    /**
     * @brief Add joint wrenches to tau
     * 
     * @param id 
     * @param jointWrench 
     */
    template<typename Derived>
    void setJointWrenches(const Eigen::MatrixBase<Derived>& jointWrench)
    {
        debugger.addEntry("add to tau", jointWrench);
        tau.noalias() += jointWrench;
        debugger.addEntry("new tau", tau);
    }

    const Eigen::Vector6d& getXiAbs() const
    {
        return xiAbs;
    }

    /**
     * @brief publish thruster commands
     * 
     * @param thrusterOutputs double pointer to first element of double array with size four
     */
    void stopThrusters()
    {
        mavros_msgs::AttitudeTarget msg;
        msg.header.stamp = ros::Time::now();
        msg.body_rate.x = 0;
        msg.body_rate.y = 0;
        msg.body_rate.z = 0;
        msg.thrust      = 0;
        pub.publish(msg);
    }

    /**
     * @brief publish thruster commands
     * 
     * @param thrusterOutputs double pointer to first element of double array with size four
     */
    void setThrusters(const double* thrusterOutputs)
    {
        mavros_msgs::AttitudeTarget msg;
        msg.header.stamp = ros::Time::now();
        std::array<double, 4> thrusterCommands = thrusterModel.calcThrusterCommands(thrusterOutputs);
        // negative signs needed because of mavros frame conversion frd <-> flu
        msg.body_rate.x =  thrusterCommands[0];
        msg.body_rate.y = -thrusterCommands[1];
        msg.body_rate.z = -thrusterCommands[2];
        msg.thrust      =  thrusterCommands[3];
        pub.publish(msg);
        debugger.addEntry("thrusters", thrusterCommands.data(), thrusterCommands.size());
        debugger.addEntry("forces", thrusterModel.Psi*Eigen::Vector4d(thrusterOutputs));
        debugger.publish();
    }

    /**
     * @brief update Phi, Theta, A, theta, zeta
     * 
     * @param newState 
     */
    virtual void updateVehicleState(const std::shared_ptr<StateProvider> newState) = 0;

    virtual void calcDecoupledWrenches(const std::shared_ptr<StateProvider> desiredState) = 0;

    virtual void calcEta(Eigen::Ref<Eigen::VectorXd> eta, const int idx) = 0;

    // first step of calculating B recursively
    virtual void calcB(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 4>> B,
                       const std::vector<int>& idxList) const = 0;

    virtual const Eigen::Vector6d& getBeta() const = 0;
    virtual const Eigen::Vector6d& getBetaDot() const = 0;

    /**
     * @brief calculate off-diagonal block matrices of motor force map
     * 
     * @param B 
     * @param X 
     */
    virtual void calcOffDiagB(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 4>> B,
                              Eigen::Matrix<double, 6, 4>& X,
                              const std::vector<int>& idxList) const = 0;

    virtual uint getDof() const = 0;
};


#endif  // VEHICLECONTROLLER_H