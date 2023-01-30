#ifndef VEHICLEESTIMATOR_H
#define VEHICLEESTIMATOR_H


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <hippo_chain/include/common/ConfigProvider.h>
#include <hippo_chain/include/common/typedefs.h>
#include <hippo_chain/include/common/sharedAlgorithms.hpp>
#include <hippo_chain/ChainVehicleState.h>
#include <hippo_chain/include/common/CustomExceptions.h>


class VehicleEstimator
{
protected:
    const int PUBLIC_ID;
    std::shared_ptr<ros::NodeHandle> nh;
    ros::Subscriber sub;
    ros::Timer timer;
    std::shared_ptr<ConfigProvider> configProvider;

    geometry_msgs::PoseWithCovariance absPose;
    geometry_msgs::TwistWithCovariance absTwist;


private:
    bool timedOut;


public:
    VehicleEstimator(const std::string& name)
    : PUBLIC_ID(shared::getID(name))
    , nh(new ros::NodeHandle(name))
    , sub(nh->subscribe("ground_truth/odom", 1, &VehicleEstimator::callback, this))
    , timer(nh->createTimer(ros::Rate(10.0), &VehicleEstimator::bark, this, true, false))
    , configProvider(new ConfigProvider(nh))
    , timedOut(true)
    {}

    void callback(const nav_msgs::Odometry& msg)
    {
        timer.stop();
        absPose = msg.pose;
        absTwist = msg.twist;
        timedOut = false;
        timer.start();
    }

    void bark(const ros::TimerEvent& e)
    {
        timedOut = true;
        throw timeout_error(sub.getTopic());
    }


    bool isTimedOut()
    {
        return timedOut;
    }


    const geometry_msgs::PoseWithCovariance& getAbsPose() const
    {
        return absPose;
    }

    const geometry_msgs::TwistWithCovariance& getAbsTwist() const
    {
        return absTwist;
    }

    virtual hippo_chain::ChainVehicleState getStateMsg() const = 0;


    void estimate()
    {
        if (timedOut) throw timeout_error(sub.getTopic());
        estimateImpl();
    }

    virtual void estimateImpl() = 0;
};


#endif  // VEHICLEESTIMATOR_H