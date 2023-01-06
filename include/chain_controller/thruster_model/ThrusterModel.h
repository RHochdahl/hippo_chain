#ifndef THRUSTERMODEL_H
#define THRUSTERMODEL_H


#include <hippo_chain/include/common/typedefs.h>
#include <hippo_chain/include/common/ConfigProvider.h>
#include <hippo_chain/include/common/sharedAlgorithms.hpp>
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

    /**
     * @brief publish thruster commands
     * 
     * @param output_it double pointer to first element of double array with size four
     */
    static inline std::array<double, 4> calcThrusterCommands(const double* output_it)
    {
        const double* const end = output_it + 4;

        std::array<double, 4> thrusterCommands;
        double* arr_it = thrusterCommands.data();
        for (; output_it!=end; output_it++, arr_it++) {
            assert(output_it != NULL);
            *arr_it = std::sqrt(std::abs(*output_it)) * shared::sgn(*output_it);
        }
        return thrusterCommands;
    }
};


#endif  // THRUSTERMODEL_H