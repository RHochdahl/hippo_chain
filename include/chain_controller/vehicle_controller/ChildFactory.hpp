#ifndef CHILDFACTORY_H
#define CHILDFACTORY_H


#include <memory>
#include "ChildVehicleController.h"
#include "../joint_model/RevoluteJointModel.h"


namespace ChildFactory
{
    std::shared_ptr<VehicleController> bearChild(const std::shared_ptr<VehicleController> ptr, const std::string& name, const int id, const std::string& jointType)
    {
        if (jointType == RevoluteJointModel::jointTypeName)
            return std::make_shared<ChildVehicleController<RevoluteJointModel>>(ptr, name, id);

        /* Add others like
        if (jointType == SomeOtherJointModel::jointTypeName)
            return std::make_shared<ChildVehicleController<SomeOtherJointModel>(ptr, name, id);
        */

        throw std::invalid_argument(("Could not find matching joint class for '" + jointType + "'!").c_str());
    }
}

#endif  // CHILDFACTORY_H
