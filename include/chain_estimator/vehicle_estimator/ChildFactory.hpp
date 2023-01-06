#ifndef CHILDESTIMATORFACTORY_H
#define CHILDESTIMATORFACTORY_H


#include <memory>
#include "ChildVehicleEstimator.h"
#include <hippo_chain/include/chain_estimator/joint_model/RevoluteJointModel.h>


namespace ChildFactory
{
    std::shared_ptr<VehicleEstimator> bearChild(const std::shared_ptr<VehicleEstimator> ptr, const std::string& name, const std::string& jointType)
    {
        if (jointType == RevoluteJointModel::jointTypeName)
            return std::make_shared<ChildVehicleEstimator<RevoluteJointModel>>(ptr, name);

        /* Add others like
        if (jointType == SomeOtherJointModel::jointTypeName)
            return std::make_shared<ChildVehicleEstimator<SomeOtherJointModel>(ptr, name);
        */

        throw std::invalid_argument(("Could not find matching joint class for '" + jointType + "'!").c_str());
    }
}

#endif  // CHILDESTIMATORFACTORY_H
