#ifndef VEHICLEMODEL_H
#define VEHICLEMODEL_H


#include <memory>
#include <math.h>
#include <Eigen/Dense>
#include <hippo_chain/include/common/typedefs.h>
#include <hippo_chain/include/common/sharedAlgorithms.hpp>
#include <hippo_chain/include/common/ConfigProvider.h>
#include <hippo_chain/include/common/Debugger.h>


#define USE_MODEL
#define USE_ORIGINAL_DESIGN
// #define USE_ACTUAL_VELOCITY
// #define USE_RIGID_MASS_FACTOR


class VehicleModel
{
private:
    double rigidMass;
    Eigen::Vector3d inertia;
    Eigen::Vector3d addedMass;
    Eigen::Vector6d inertias;
    Eigen::Vector6d linDragCoeff;
    Eigen::Vector6d quadDragCoeff;


public:
    VehicleModel(const std::shared_ptr<ConfigProvider>& configProvider)
    {
        const std::string nsExtension = "vehicle_model/";
        const std::string poseStr[] = {"x", "y", "z", "roll", "pitch", "yaw"};

        // don't use iterators as performance is irrelevant here

        rigidMass = configProvider->getValuePositiveWithDefault(nsExtension + "rigid_mass", 0.0);
        for (int i=0; i<3; i++) {
            addedMass(i) = configProvider->getValuePositiveWithDefault(nsExtension + "added_mass/" + poseStr[i], 0.0);
            inertias(i) = rigidMass + addedMass(i);
        }

#ifndef USE_RIGID_MASS_FACTOR
        const double maxAddedMass = addedMass.topRows<3>().maxCoeff();
        addedMass.topRows<3>() -= maxAddedMass * Eigen::Vector3d::Ones();
        rigidMass -= maxAddedMass;
#endif  // USE_RIGID_MASS_FACTOR

        for (int i=3; i<6; i++) {
            inertias(i) = inertia(i-3) = configProvider->getValuePositiveWithDefault(nsExtension + "rigid_inertia/" + poseStr[i], 0.0)
                                       + configProvider->getValuePositiveWithDefault(nsExtension + "added_inertia/" + poseStr[i], 0.0);
        }

        for (int i=0; i<6; i++) {
            linDragCoeff(i) = configProvider->getValuePositiveWithDefault(nsExtension + "drag/linear/" + poseStr[i], 0.0);
            quadDragCoeff(i) = configProvider->getValuePositiveWithDefault(nsExtension + "drag/quadratic/" + poseStr[i], 0.0);
        }
    }

    ~VehicleModel()
    {}


    Eigen::Vector6d calcWrenches(const Eigen::Vector6d& absVel,
                                 const Eigen::Vector6d& beta,
                                 const Eigen::Vector6d& dbeta,
                                 const double k=-1,
                                 Debugger* debugger=NULL) const
    {
        // zero net buoyancy is assumed

        Eigen::Vector6d result;

#ifdef USE_MODEL
#ifdef USE_ORIGINAL_DESIGN

        // coriolis wrenches
        const Eigen::Vector6d momentum = inertias.cwiseProduct(absVel);

        result.topRows<3>().noalias()       = beta.bottomRows<3>().cross(momentum.topRows<3>());
        result.bottomRows<3>().noalias()    = beta.bottomRows<3>().cross(momentum.bottomRows<3>())
                                            + beta.topRows<3>().cross(momentum.topRows<3>());

        // drag wrenches
        result.noalias() += linDragCoeff.cwiseProduct(beta);
        result.noalias() += quadDragCoeff.cwiseProduct(beta).cwiseProduct(absVel.cwiseAbs());

#else  // USE_ORIGINAL_DESIGN
#ifndef USE_ACTUAL_VELOCITY
        // coriolis wrenches
        const Eigen::Vector3d addedLinearMomentum = addedMass.cwiseProduct(absVel.topRows<3>());
        const Eigen::Vector3d angularMomentum = inertia.cwiseProduct(absVel.bottomRows<3>());

        result.topRows<3>().noalias()       = beta.bottomRows<3>().cross(addedLinearMomentum);
        result.bottomRows<3>().noalias()    = beta.bottomRows<3>().cross(angularMomentum)
                                            + beta.topRows<3>().cross(addedLinearMomentum);

#ifdef USE_RIGID_MASS_FACTOR
        result.topRows<3>().noalias()       += rigidMass * ((k+1) * absVel.bottomRows<3>().cross(beta.topRows<3>())
                                                              + k * absVel.topRows<3>().cross(beta.bottomRows<3>()));
        result.bottomRows<3>().noalias()    += k * rigidMass * absVel.topRows<3>().cross(beta.topRows<3>());

#else  // USE_RIGID_MASS_FACTOR
        result.topRows<3>().noalias()       += rigidMass * absVel.bottomRows<3>().cross(beta.topRows<3>());
#endif  // USE_RIGID_MASS_FACTOR

#ifndef NDEBUG
        if (debugger) {
            debugger->addEntry("coriolis wrenches", result);
            debugger->addEntry("inertial wrenches", inertias.cwiseProduct(dbeta));
            debugger->addEntry("linear drag", linDragCoeff.cwiseProduct(beta));
            debugger->addEntry("quadratic drag", quadDragCoeff.cwiseProduct(beta).cwiseProduct(absVel.cwiseAbs()));
        }
#endif  // NDEBUG

        // drag wrenches
        result.noalias() += linDragCoeff.cwiseProduct(beta);
        result.noalias() += quadDragCoeff.cwiseProduct(beta).cwiseProduct(absVel.cwiseAbs());

#else  // USE_ACTUAL_VELOCITY

        // coriolis wrenches
        const Eigen::Vector6d momentum = inertias.cwiseProduct(absVel);

        result.topRows<3>().noalias()       = absVel.bottomRows<3>().cross(momentum.topRows<3>());
        result.bottomRows<3>().noalias()    = absVel.bottomRows<3>().cross(momentum.bottomRows<3>())
                                            + absVel.topRows<3>().cross(momentum.topRows<3>());

        // drag wrenches
        result.noalias() += linDragCoeff.cwiseProduct(absVel);
        result.noalias() += quadDragCoeff.cwiseProduct(absVel).cwiseProduct(absVel.cwiseAbs());

#endif  // USE_ACTUAL_VELOCITY
#endif  // USE_ORIGINAL_DESIGN

        // inertial wrenches
        result.noalias() += inertias.cwiseProduct(dbeta);

#else  // USE_MODEL
        result.setZero();
#endif  // USE_MODEL

        return result;
    }
};


#endif  // VEHICLEMODEL_H