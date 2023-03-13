#ifndef BASEVEHICLEESTIMATOR_H
#define BASEVEHICLEESTIMATOR_H


#include "VehicleEstimator.h"


class BaseVehicleEstimator : public VehicleEstimator
{
public:
    BaseVehicleEstimator(const std::string& name)
    : VehicleEstimator(name)
    {}

    hippo_chain::ChainVehicleState getStateMsg() const
    {
        hippo_chain::ChainVehicleState msg;

        msg.pose = {absPose.pose.position.x,
                    absPose.pose.position.y,
                    absPose.pose.position.z,
                    absPose.pose.orientation.w,
                    absPose.pose.orientation.x,
                    absPose.pose.orientation.y,
                    absPose.pose.orientation.z};

        msg.twist = {absTwist.twist.linear.x,
                     absTwist.twist.linear.y,
                     absTwist.twist.linear.z,
                     absTwist.twist.angular.x,
                     absTwist.twist.angular.y,
                     absTwist.twist.angular.z};

        msg.vehicle_id = PUBLIC_ID;

        return msg;
    }

    void estimateImpl() {}
};



#endif  // BASEVEHICLEESTIMATOR_H