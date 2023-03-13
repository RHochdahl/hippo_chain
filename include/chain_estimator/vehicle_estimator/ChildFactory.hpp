#ifndef CHILDESTIMATORFACTORY_H
#define CHILDESTIMATORFACTORY_H


#include <memory>
#include "ChildVehicleEstimator.h"
#include <hippo_chain/include/chain_estimator/joint_model/RevoluteJointModel.h>
#include <hippo_chain/include/chain_estimator/joint_model/UniversalJointModel.h>


namespace ChildFactory
{
    std::shared_ptr<VehicleEstimator> bearChild(const std::shared_ptr<VehicleEstimator> parent, const std::string& name, const std::string& jointType)
    {
        if (jointType == RevoluteJointModel::jointTypeName)
            return std::make_shared<ChildVehicleEstimator<RevoluteJointModel>>(parent, name);

        if (jointType == UniversalJointModel::jointTypeName)
            return std::make_shared<ChildVehicleEstimator<UniversalJointModel>>(parent, name);

        /* Add others like
        if (jointType == SomeOtherJointModel::jointTypeName)
            return std::make_shared<ChildVehicleEstimator<SomeOtherJointModel>(parent, name);
        */

        throw std::invalid_argument(("Could not find matching joint class for '" + jointType + "'!").c_str());
    }
}

#endif  // CHILDESTIMATORFACTORY_H
