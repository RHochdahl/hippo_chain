#ifndef CHILDVEHICLEESTIMATOR_H
#define CHILDVEHICLEESTIMATOR_H


#include "VehicleEstimator.h"
#include <memory>


template<typename JointModel>
class ChildVehicleEstimator : public VehicleEstimator
{
private:
    std::shared_ptr<VehicleEstimator> parent;
    JointModel jointModel;


public:
    ChildVehicleEstimator(std::shared_ptr<VehicleEstimator> parent, const std::string& name)
    : VehicleEstimator(name)
    , parent(parent)
    , jointModel(configProvider)
    {}


    hippo_chain::ChainVehicleState getStateMsg() const
    {
        hippo_chain::ChainVehicleState msg;

        msg.pose = jointModel.coordsToVector();

        msg.twist = jointModel.velToVector();

        msg.vehicle_id = PUBLIC_ID;

        return msg;
    }

    void estimateImpl()
    {
        jointModel.updateCoordinates(absPose, parent->getAbsPose());
        jointModel.updateVelocities(absTwist, parent->getAbsTwist());
        jointModel.executeFilter();
    }
};


#endif  // CHILDVEHICLEESTIMATOR_H