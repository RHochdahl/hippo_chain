#ifndef VEHICLEMODEL_H
#define VEHICLEMODEL_H


#include <memory>
#include <math.h>
#include <Eigen/Dense>
#include <hippo_chain/include/common/typedefs.h>
#include <hippo_chain/include/common/sharedAlgorithms.hpp>
#include <hippo_chain/include/common/ConfigProvider.h>


class VehicleModel
{
private:
    Eigen::Vector6d inertias;
    Eigen::Vector6d linDragCoeff;
    Eigen::Vector6d quadDragCoeff;


public:
    VehicleModel(const std::shared_ptr<ConfigProvider>& configProvider)
    {
        const std::string nsExtension = "vehicle_model/";
        const std::string poseStr[] = {"x", "y", "z", "roll", "pitch", "yaw"};

        // don't use iterators as performance is irrelevant here

        const double rigidMass = configProvider->getValuePositiveWithDefault(nsExtension + "rigid_mass", 0);
        for (int i=0; i<3; i++) {
            inertias(i) = rigidMass + configProvider->getValuePositiveWithDefault(nsExtension + "added_mass/" + poseStr[i], 0);
        }
        for (int i=3; i<6; i++) {
            inertias(i) = configProvider->getValuePositiveWithDefault(nsExtension + "rigid_inertia/" + poseStr[i], 0)
                        + configProvider->getValuePositiveWithDefault(nsExtension + "added_inertia/" + poseStr[i], 0);
        }

        for (int i=0; i<6; i++) {
            linDragCoeff(i) = configProvider->getValuePositiveWithDefault(nsExtension + "drag/linear/" + poseStr[i], 0);
            quadDragCoeff(i) = configProvider->getValuePositiveWithDefault(nsExtension + "drag/quadratic/" + poseStr[i], 0);
        }
    }

    ~VehicleModel()
    {

    }

    template<typename Derived1,
             typename Derived2>
    Eigen::Vector6d calcWrenches(const Eigen::MatrixBase<Derived1>& absVel,
                                 const Eigen::Vector6d& beta,
                                 const Eigen::MatrixBase<Derived2>& dbeta,
                                 Debugger* debugger=NULL) const
    {
        // zero net buoyancy is assumed

        Eigen::Vector6d result;
// #define USE_ACTUAL_VELOCITY
#ifndef USE_ACTUAL_VELOCITY
        // coriolis wrenches
        const Eigen::Vector6d momentum = inertias.cwiseProduct(absVel);
        if (debugger) debugger->addEntry("momentum", momentum);
        result.topRows<3>().noalias()       = beta.bottomRows<3>().cross(momentum.topRows<3>());
        result.bottomRows<3>().noalias()    = beta.topRows<3>().cross(momentum.topRows<3>())
                                            + beta.bottomRows<3>().cross(momentum.bottomRows<3>());

#ifndef NDEBUG
        if (debugger) debugger->addEntry("coriolis wrenches", result);
        if (debugger) debugger->addEntry("inertial wrenches", inertias.cwiseProduct(dbeta));
        if (debugger) debugger->addEntry("linear drag", linDragCoeff.cwiseProduct(beta));
        if (debugger) debugger->addEntry("quadratic drag", quadDragCoeff.cwiseProduct(beta).cwiseProduct(absVel.cwiseAbs()));
#endif  // NDEBUG

        // inertial wrenches
        result.noalias() += inertias.cwiseProduct(dbeta);

        // drag wrenches
        result.noalias() += linDragCoeff.cwiseProduct(beta);
        result.noalias() += quadDragCoeff.cwiseProduct(beta).cwiseProduct(absVel.cwiseAbs());
#else  // USE_ACTUAL_VELOCITY
        Eigen::Vector6d absVelVec = absVel;

        // coriolis wrenches
        const Eigen::Vector6d momentum = inertias.cwiseProduct(absVelVec);
        if (debugger) debugger->addEntry("momentum", momentum);
        result.topRows<3>().noalias()       = absVelVec.bottomRows<3>().cross(momentum.topRows<3>());
        result.bottomRows<3>().noalias()    = absVelVec.topRows<3>().cross(momentum.topRows<3>())
                                            + absVelVec.bottomRows<3>().cross(momentum.bottomRows<3>());

#ifndef NDEBUG
        if (debugger) debugger->addEntry("coriolis wrenches", result);
        if (debugger) debugger->addEntry("linear drag", linDragCoeff.cwiseProduct(absVelVec));
        if (debugger) debugger->addEntry("quadratic drag", quadDragCoeff.cwiseProduct(absVelVec).cwiseProduct(absVelVec.cwiseAbs()));
#endif  // NDEBUG

        // drag wrenches
        result.noalias() += linDragCoeff.cwiseProduct(absVelVec);
        result.noalias() += quadDragCoeff.cwiseProduct(absVelVec).cwiseProduct(absVelVec.cwiseAbs());
#endif  // USE_ACTUAL_VELOCITY
//        result.setZero();
        return result;
    }
};


#endif  // VEHICLEMODEL_H