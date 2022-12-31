#ifndef BASEVEHICLECONTROLLER_H
#define BASEVEHICLECONTROLLER_H


#include "VehicleController.h"
#include <hippo_chain/BaseControllerConfig.h>


class BaseVehicleController : public VehicleController
{
private:
    struct ControllerParam {
        double kSigma1;
        double kSigma2;
        double kP;
        double kSat;
        double lim;
    } param;

    struct ControllerStates {
        Eigen::Vector7d desiredPose;
        Eigen::Vector6d desiredTwist;
        Eigen::Vector6d sigma;
        Eigen::Vector6d sigmaDot;
    } controllerStates;

    Eigen::Vector7d poseAbs;

    dynamic_reconfigure::Server<hippo_chain::BaseControllerConfig> server;
    dynamic_reconfigure::Server<hippo_chain::BaseControllerConfig>::CallbackType f;


    void updateControlParameters(hippo_chain::BaseControllerConfig &config, uint32_t level)
    {
        param.kSigma1 = config.kSigma1;
        param.kSigma2 = config.kSigma2;
        param.kP = config.kP;
        param.kSat = config.kSat;
        param.lim = config.lim;
        ROS_INFO("Updated controller parameters for '%s'", nh->getNamespace().c_str());
    }

    /**
     * @brief calculates sigma
     * 
     * @param desiredState desired pose and twist in map coordinates
     */
    void calcSigma(const std::shared_ptr<StateProvider> desiredState)
    {
        controllerStates.desiredPose = desiredState->getPose<Eigen::Vector7d>();
        controllerStates.desiredTwist = desiredState->getTwist<Eigen::Vector6d>();

        if (!shared::isUnitQuaternion(controllerStates.desiredPose.bottomRows(4))) throw quaternion_error();
        if (!shared::isUnitQuaternion(poseAbs.bottomRows(4))) throw quaternion_error();

        const double rQuatDes = controllerStates.desiredPose[3];
        const Eigen::Vector3d iQuatDes = controllerStates.desiredPose.bottomRows(3);
        const double rQuatAct = poseAbs[3];
        const Eigen::Vector3d iQuatAct = poseAbs.bottomRows(3);

        // rotation matrix R^1_0 or S_K'K to rotate vector from world to base frame
        const Eigen::Matrix3d R_1_0 = Eigen::Quaterniond(poseAbs[3], -poseAbs[4], -poseAbs[5], -poseAbs[6]).toRotationMatrix();

        const Eigen::Vector3d posErr = R_1_0 * (controllerStates.desiredPose.topRows(3) - poseAbs.topRows(3));
        const double etaErr = rQuatAct*rQuatDes + iQuatAct.dot(iQuatDes);
        const Eigen::Vector3d epsilonErr = (etaErr > 0) ? (rQuatAct * iQuatDes - rQuatDes * iQuatAct + shared::cross3(iQuatDes, iQuatAct))
                                                        : (rQuatDes * iQuatAct - rQuatAct * iQuatDes + shared::cross3(iQuatAct, iQuatDes));

        const Eigen::Vector3d xiLinDes = R_1_0 * controllerStates.desiredTwist.topRows(3);
        const Eigen::Vector3d xiAngDes = R_1_0 * controllerStates.desiredTwist.bottomRows(3);

        controllerStates.sigma.topRows(3) = xiLinDes + param.kSigma1 * posErr;
        controllerStates.sigma.bottomRows(3) = xiAngDes + param.kSigma2 * epsilonErr;

        // Assumption: des twist in world frame, actual twist in base frame
        const Eigen::Vector3d posErrDot = xiLinDes - xiAbs.topRows(3);
        const Eigen::Vector3d omegaErr = xiAngDes - xiAbs.bottomRows(3);
        const Eigen::Vector3d epsilonErrDot = 0.5 * (etaErr*omegaErr + shared::cross3(epsilonErr, omegaErr));

        const Eigen::Vector3d xiLinDesDot = shared::cross3(xiLinDes, xiAbs.bottomRows(3));
        const Eigen::Vector3d xiAngDesDot = shared::cross3(xiAngDes, xiAbs.bottomRows(3));

        controllerStates.sigmaDot.topRows(3) = xiLinDesDot + param.kSigma1 * posErrDot;
        controllerStates.sigmaDot.bottomRows(3) = xiAngDesDot + param.kSigma2 * epsilonErrDot;
    }


public:
    BaseVehicleController(const std::string& name)
    : VehicleController(name)
    , server(*nh)
    , f(boost::bind(&BaseVehicleController::updateControlParameters, this, _1, _2))
    {
        server.setCallback(f);
        ROS_INFO("Constructed base vehicle '%s'", name.c_str());
    }

    ~BaseVehicleController()
    {}

    /**
     * @brief update state vector
     * 
     * @param newState vehicle pose in map coordinates and twist in base coordinates
     */
    void updateVehicleState(const std::shared_ptr<StateProvider> newState)
    {
        if (newState == NULL) throw auto_print_error("New state is not initialized!");

        poseAbs = newState->getPose<Eigen::Vector7d>();
        xiAbs = newState->getTwist<Eigen::Vector6d>();
    }

    /**
     * @brief calculates desired wrenches on this vehicle (joint forces not yet applied)
     * 
     * @param desiredState desired pose and twist in map coordinates
     */
    void calcDecoupledWrenches(const std::shared_ptr<StateProvider> desiredState)
    {
        if (desiredState == NULL) throw auto_print_error("Desired state is not initialized!");

        calcSigma(desiredState);
        tau = vehicleModel.calcWrenches(xiAbs, controllerStates.sigma, controllerStates.sigmaDot);
    }

    void calcB(std::vector<std::pair<uint, Eigen::Matrix<double, Eigen::Dynamic, 4>>>& B) const
    {
        B.push_back(std::make_pair(ID, thrusterModel.Psi));
    }

    /**
     * @brief calculate off-diagonal block matrices of motor force map
     * 
     * @param B 
     * @param X 
     */
    void calcOffDiagB(std::vector<std::pair<uint, Eigen::Matrix<double, Eigen::Dynamic, 4>>>& B, Eigen::Matrix<double, 6, 4>& X) const
    {
        B.push_back(std::make_pair(ID, X));
    }

    Eigen::VectorXd calcEta() const
    {
        const Eigen::Vector6d s = controllerStates.sigma - xiAbs;
        return tau + param.kP * s + param.kSat * s / std::max(s.norm(), param.lim);
    }

    const Eigen::Vector6d& getBeta() const
    {
        return controllerStates.sigma;
    }

    const Eigen::Vector6d& getBetaDot() const
    {
        return controllerStates.sigmaDot;
    }

    uint getDof() const
    {
        return 6;
    }
};


#endif  // BASEVEHICLECONTROLLER_H