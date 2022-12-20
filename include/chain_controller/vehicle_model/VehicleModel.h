#ifndef VEHICLEMODEL_H
#define VEHICLEMODEL_H


#include <memory>
#include <math.h>
#include <Eigen/Dense>
#include "../../utils/typedefs.h"
#include "../../utils/sharedAlgorithms.hpp"
#include "../../utils/ConfigProvider.h"


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

    Eigen::Vector6d calcWrenches(const Eigen::Vector6d& absVel, const Eigen::Vector6d& beta, const Eigen::Vector6d& dbeta) const
    {
        // zero net buoyancy is assumed

        const Eigen::Vector6d inertiaWrenches = inertias.cwiseProduct(dbeta);

        const Eigen::Vector6d momentum = inertias.cwiseProduct(absVel);

        // coriolis and centrifugal wrenches
        Eigen::Vector6d ccWrenches;
        ccWrenches.topRows(3) = shared::cross3(beta.bottomRows(3), momentum.topRows(3));
        ccWrenches.bottomRows(3) = shared::cross3(beta.topRows(3), momentum.topRows(3)) + shared::cross3(beta.bottomRows(3), momentum.bottomRows(3));

        const Eigen::Vector6d dragWrenches = linDragCoeff.cwiseProduct(beta) + quadDragCoeff.cwiseProduct(beta).cwiseProduct(absVel.cwiseAbs());

        return inertiaWrenches + ccWrenches + dragWrenches;
    }
};


#endif  // VEHICLEMODEL_H