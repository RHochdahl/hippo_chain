#ifndef VEHICLEMODEL_H
#define VEHICLEMODEL_H


#include <memory>
#include <math.h>
#include <Eigen/Dense>
#include "../../common/typedefs.h"
#include "../../common/sharedAlgorithms.hpp"
#include "../../common/ConfigProvider.h"


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
                                 const Eigen::MatrixBase<Derived2>& dbeta) const
    {
        // zero net buoyancy is assumed

        Eigen::Vector6d result;

        // coriolis wrenches
        const Eigen::Vector6d momentum = inertias.cwiseProduct(absVel);
        result.topRows<3>().noalias()       = beta.bottomRows<3>().cross(momentum.topRows<3>());
        result.bottomRows<3>().noalias()    = beta.topRows<3>().cross(momentum.topRows<3>())
                                            + beta.bottomRows<3>().cross(momentum.bottomRows<3>());

        // inertial wrenches
        result.noalias() += inertias.cwiseProduct(dbeta);

        // drag wrenches
        result.noalias() += linDragCoeff.cwiseProduct(beta);
        result.noalias() += quadDragCoeff.cwiseProduct(beta).cwiseProduct(absVel.cwiseAbs());

        return result;
    }
};


#endif  // VEHICLEMODEL_H