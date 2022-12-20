#ifndef THRUSTERMODEL_H
#define THRUSTERMODEL_H


#include "../../utils/typedefs.h"
#include "../../utils/sharedAlgorithms.hpp"
#include "../../utils/ConfigProvider.h"
#include <string>
#include <array>


class ThrusterModel
{
private:
    struct Thruster {
        double maxForce;
        double maxTorque;
        bool cw;
        double ry;
        double rz;
    };

    const std::array<Thruster, 4> thruster;


    static inline std::array<Thruster, 4> initThrusterArray(const std::shared_ptr<ConfigProvider>& configProvider)
    {
        std::array<Thruster, 4> tempArr;
        const std::string nsExtension = "thruster_model/";

        for (int i=0; i<4; i++) {
            const std::string prefix = nsExtension + std::to_string(i) + "/";
            if (!(configProvider->getAbsValue(prefix + "max_force", tempArr[i].maxForce) &&
                  configProvider->getAbsValue(prefix + "max_torque", tempArr[i].maxTorque) &&
                  configProvider->getValue(prefix + "clockwise", tempArr[i].cw) &&
                  configProvider->getValue(prefix + "pos_y", tempArr[i].ry) &&
                  configProvider->getValue(prefix + "pos_z", tempArr[i].rz)))
                ROS_FATAL("Thruster parameters could not be retrieved!");
        }

        return tempArr;
    }

    static inline Eigen::Matrix<double, 6, 4> initPsi(const std::array<Thruster, 4>& thrusterArray)
    {
        Eigen::Matrix<double, 6, 4> tempPsi = Eigen::Matrix<double, 6, 4>::Zero();

        for (int i=0; i<4; i++) {
            tempPsi(0, i) = thrusterArray[i].cw ? thrusterArray[i].maxForce : -thrusterArray[i].maxForce;
            tempPsi(3, i) = thrusterArray[i].cw ? thrusterArray[i].maxTorque : -thrusterArray[i].maxTorque;
            tempPsi(4, i) = thrusterArray[i].rz * tempPsi(0, i);
            tempPsi(5, i) = -thrusterArray[i].ry * tempPsi(0, i);
        }

        return tempPsi;
    }

public:
    const Eigen::Matrix<double, 6, 4> Psi;    // thruster config


    ThrusterModel(const std::shared_ptr<ConfigProvider>& configProvider)
    : thruster(initThrusterArray(configProvider))
    , Psi(initPsi(thruster))
    {}

    ~ThrusterModel()
    {}

    static inline std::array<double, 4> calcThrusterCommands(const Eigen::Vector4d& thrusterOutputs)
    {
        std::array<double, 4> thrusterCommands;
        for (int idx=0; idx<thrusterCommands.size(); idx++) {
            double thrusterOutput = thrusterOutputs(idx);
            thrusterCommands[idx] = std::sqrt(std::abs(thrusterOutput)) * shared::sgn(thrusterOutput);
        }
        return thrusterCommands;
    }
};


#endif  // THRUSTERMODEL_H