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


class VehicleModel
{
private:
    struct VehicleParams {
        double rigidMass;
        Eigen::Vector3d inertia;
        Eigen::Vector3d addedMass;
        Eigen::Vector6d inertias;
        Eigen::Vector6d linDragCoeff;
        Eigen::Vector6d quadDragCoeff;
    } params;

    struct VehicleState {
        Eigen::Vector6d absVel;
        Eigen::Vector6d buoyancyForce;
    } state;


public:
    VehicleModel(const std::shared_ptr<ConfigProvider>& configProvider)
    {
        const std::string nsExtension = "vehicle_model/";
        const std::string poseStr[] = {"x", "y", "z", "roll", "pitch", "yaw"};

        // don't use iterators as performance is irrelevant here

        params.rigidMass = configProvider->getValuePositiveWithDefault(nsExtension + "rigid_mass", 0.0);
        for (int i=0; i<3; i++) {
            params.addedMass(i) = configProvider->getValuePositiveWithDefault(nsExtension + "added_mass/" + poseStr[i], 0.0);
            params.inertias(i) = params.rigidMass + params.addedMass(i);
        }

        for (int i=3; i<6; i++) {
            params.inertias(i) = params.inertia(i-3) = configProvider->getValuePositiveWithDefault(nsExtension + "rigid_inertia/" + poseStr[i], 0.0)
                                                     + configProvider->getValuePositiveWithDefault(nsExtension + "added_inertia/" + poseStr[i], 0.0);
        }

        for (int i=0; i<6; i++) {
            params.linDragCoeff(i) = configProvider->getValuePositiveWithDefault(nsExtension + "drag/linear/" + poseStr[i], 0.0);
            params.quadDragCoeff(i) = configProvider->getValuePositiveWithDefault(nsExtension + "drag/quadratic/" + poseStr[i], 0.0);
        }

        state.absVel.setZero();
        state.buoyancyForce.setZero();
    }

    ~VehicleModel()
    {}


    virtual void setOrientation(const Eigen::Matrix3d R)
    {}

    void setVelocity(const Eigen::Vector6d& absVel)
    {
        state.absVel = absVel;
    }

    virtual Eigen::Vector6d calcWrenches(const Eigen::Vector6d& vel,
                                         const Eigen::Vector6d& acc,
                                         Debugger* debugger=NULL) const
    {
        // zero net buoyancy is assumed

        Eigen::Vector6d result;

#ifdef USE_MODEL
#ifdef USE_ORIGINAL_DESIGN

        // coriolis wrenches
        const Eigen::Vector6d momentum = params.inertias.cwiseProduct(state.absVel);

        result.topRows<3>().noalias()       = vel.bottomRows<3>().cross(momentum.topRows<3>());
        result.bottomRows<3>().noalias()    = vel.bottomRows<3>().cross(momentum.bottomRows<3>())
                                            + vel.topRows<3>().cross(momentum.topRows<3>());

#ifndef NDEBUG
        if (debugger) {
            debugger->addEntry("coriolis wrenches", result);
            debugger->addEntry("inertial wrenches", params.inertias.cwiseProduct(acc));
            debugger->addEntry("linear drag", params.linDragCoeff.cwiseProduct(vel));
            debugger->addEntry("quadratic drag", params.quadDragCoeff.cwiseProduct(vel).cwiseProduct(state.absVel.cwiseAbs()));
        }
#endif  // NDEBUG

        // drag wrenches
        result.noalias() += params.linDragCoeff.cwiseProduct(vel);
        result.noalias() += params.quadDragCoeff.cwiseProduct(vel).cwiseProduct(state.absVel.cwiseAbs());

#else  // USE_ORIGINAL_DESIGN
#ifndef USE_ACTUAL_VELOCITY
        // coriolis wrenches
        const Eigen::Vector3d addedLinearMomentum = params.addedMass.cwiseProduct(state.absVel.topRows<3>());
        const Eigen::Vector3d angularMomentum = params.inertia.cwiseProduct(state.absVel.bottomRows<3>());

        result.topRows<3>().noalias()       = vel.bottomRows<3>().cross(addedLinearMomentum);
                                            + params.rigidMass * state.absVel.bottomRows<3>().cross(vel.topRows<3>());
        result.bottomRows<3>().noalias()    = vel.bottomRows<3>().cross(angularMomentum)
                                            + vel.topRows<3>().cross(addedLinearMomentum);

#ifndef NDEBUG
        if (debugger) {
            debugger->addEntry("coriolis wrenches", result);
            debugger->addEntry("inertial wrenches", params.inertias.cwiseProduct(acc));
            debugger->addEntry("linear drag", params.linDragCoeff.cwiseProduct(vel));
            debugger->addEntry("quadratic drag", params.quadDragCoeff.cwiseProduct(vel).cwiseProduct(state.absVel.cwiseAbs()));
        }
#endif  // NDEBUG

        // drag wrenches
        result.noalias() += params.linDragCoeff.cwiseProduct(vel);
        result.noalias() += params.quadDragCoeff.cwiseProduct(vel).cwiseProduct(state.absVel.cwiseAbs());

#else  // USE_ACTUAL_VELOCITY

        // coriolis wrenches
        const Eigen::Vector6d momentum = params.inertias.cwiseProduct(state.absVel);

        result.topRows<3>().noalias()       = state.absVel.bottomRows<3>().cross(momentum.topRows<3>());
        result.bottomRows<3>().noalias()    = state.absVel.bottomRows<3>().cross(momentum.bottomRows<3>())
                                            + state.absVel.topRows<3>().cross(momentum.topRows<3>());

        // drag wrenches
        result.noalias() += params.linDragCoeff.cwiseProduct(state.absVel);
        result.noalias() += params.quadDragCoeff.cwiseProduct(state.absVel).cwiseProduct(state.absVel.cwiseAbs());

#endif  // USE_ACTUAL_VELOCITY
#endif  // USE_ORIGINAL_DESIGN

        // inertial wrenches
        result.noalias() += params.inertias.cwiseProduct(acc);

#else  // USE_MODEL
        result.setZero();
#endif  // USE_MODEL

        return result;
    }
};


#endif  // VEHICLEMODEL_H