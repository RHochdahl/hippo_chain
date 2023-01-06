#ifndef VEHICLECONTROLLER_H
#define VEHICLECONTROLLER_H


#include <ros/ros.h>
#include <math.h>
#include <Eigen/Dense>

#include <dynamic_reconfigure/server.h>

#include <hippo_chain/include/chain_controller/vehicle_model/VehicleModel.h>
#include <hippo_chain/include/chain_controller/state/StateProvider.h>
#include <hippo_chain/include/chain_controller/thruster_model/ThrusterModel.h>
#include <hippo_chain/include/common/typedefs.h>
#include <hippo_chain/include/common/sharedAlgorithms.hpp>
#include <mavros_msgs/AttitudeTarget.h>


class VehicleController
{
public:
    const int ID;

protected:
    std::shared_ptr<ros::NodeHandle> nh;
    ros::Publisher pub;
    std::shared_ptr<ConfigProvider> configProvider;

    VehicleModel vehicleModel;
    ThrusterModel thrusterModel;

    Eigen::Vector6d tau;    // desired wrenches

    Eigen::Vector6d xiAbs;  // abs velocity


public:
    VehicleController(const std::string& name, const int id)
    : ID(id)
    , nh(new ros::NodeHandle(name))
    , pub(nh->advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 1))
    , configProvider(new ConfigProvider(nh))
    , vehicleModel(configProvider)
    , thrusterModel(configProvider)
    {}

    ~VehicleController()
    {
        if (ros::ok()) {
            stopThrusters();
        }
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
        tau.noalias() += jointWrench;
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
    void stopThrusters() const
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
    void setThrusters(const double* thrusterOutputs) const
    {
        mavros_msgs::AttitudeTarget msg;
        msg.header.stamp = ros::Time::now();
        std::array<double, 4> thrusterCommands = thrusterModel.calcThrusterCommands(thrusterOutputs);        
        msg.body_rate.x = thrusterCommands[0];
        msg.body_rate.y = thrusterCommands[1];
        msg.body_rate.z = thrusterCommands[2];
        msg.thrust      = thrusterCommands[3];
        pub.publish(msg);
    }

    /**
     * @brief update Phi, Theta, A, theta, zeta
     * 
     * @param newState 
     */
    virtual void updateVehicleState(const std::shared_ptr<StateProvider> newState) = 0;

    virtual void calcDecoupledWrenches(const std::shared_ptr<StateProvider> desiredState) = 0;

    virtual void calcEta(Eigen::Ref<Eigen::VectorXd> eta, const int idx) const = 0;

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