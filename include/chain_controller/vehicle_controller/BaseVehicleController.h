#ifndef BASEVEHICLECONTROLLER_H
#define BASEVEHICLECONTROLLER_H


#include "VehicleController.h"
#include <hippo_chain/FixBase.h>


#define IGNORE_Z_ERROR


class BaseVehicleController : public VehicleController
{
private:
    struct ControllerParam {
        double kSigma1;
        double kSigma2;
        double kP;
        double kSat;
        double lim;
        double maxPositionError;
        double maxQuaternionError;
    } param;

    struct ControllerStates {
        Eigen::Vector7d desiredPose;
        Eigen::Vector6d desiredTwist;
        Eigen::Vector6d sigma;
        Eigen::Vector6d sigmaDot;
    } controllerStates;

    Eigen::Vector7d poseAbs;

    bool fixed;


    void updateControlParameters(const hippo_chain::VehicleControllerConfig &config, uint32_t level)
    {
        if (!(level & DynamicReconfigureLevels::BASE)) return;
        param.kSigma1 = config.kSigma1;
        param.kSigma2 = config.kSigma2;
        param.kP = config.kP;
        param.kSat = config.kSat;
        param.lim = config.lim;
        param.maxPositionError = config.maxPositionError;
        param.maxQuaternionError = config.maxQuaternionError;
        ROS_INFO("Updated base controller parameters for '%s'", nh->getNamespace().c_str());
    }

    /**
     * @brief calculates sigma
     * 
     * @param desiredState desired pose and twist in map coordinates
     */
    void calcSigma(const std::shared_ptr<StateProvider> desiredState)
    {
        if (fixed) {
            controllerStates.sigma.fill(0);
            controllerStates.sigmaDot.fill(0);
            return;
        }

        controllerStates.desiredPose = desiredState->getPose<Eigen::Vector7d>();
        controllerStates.desiredTwist = desiredState->getTwist<Eigen::Vector6d>();
        debugger.addEntry("desired pose", desiredState->pose.data(), desiredState->pose.size());
        debugger.addEntry("desired twist", desiredState->twist.data(), desiredState->twist.size());

        if (!shared::isUnitQuaternion(controllerStates.desiredPose.bottomRows<4>())) throw quaternion_error();
        if (!shared::isUnitQuaternion(poseAbs.bottomRows<4>())) throw quaternion_error();

        const double rQuatDes = controllerStates.desiredPose[3];
        const Eigen::Vector3d iQuatDes = controllerStates.desiredPose.bottomRows<3>();
        const double rQuatAct = poseAbs[3];
        const Eigen::Vector3d iQuatAct = poseAbs.bottomRows<3>();

        // rotation matrix R^1_0 or S_K'K to rotate vector from world to base frame
        const Eigen::Matrix3d R_1_0 = Eigen::Quaterniond(poseAbs[3], -poseAbs[4], -poseAbs[5], -poseAbs[6]).toRotationMatrix();
        debugger.addEntry("R", R_1_0);

        const Eigen::Vector3d posErr = limitError(Eigen::Vector3d(R_1_0 * (controllerStates.desiredPose.topRows<3>() - poseAbs.topRows<3>())), param.maxPositionError);
        const int etaErrSgn = shared::sgn(rQuatAct*rQuatDes + iQuatAct.dot(iQuatDes));
        const Eigen::Vector3d epsilonErr = limitError(Eigen::Vector3d(etaErrSgn * (rQuatAct * iQuatDes - rQuatDes * iQuatAct + iQuatDes.cross(iQuatAct))), param.maxQuaternionError);;
        const double etaErr = std::sqrt(1.0 - epsilonErr.squaredNorm());
        debugger.addEntry("position error", posErr);
        debugger.addEntry("epsilon error", epsilonErr);
        debugger.addEntry("eta error", etaErr);

        const Eigen::Vector3d xiLinDes = R_1_0 * controllerStates.desiredTwist.topRows<3>();
        const Eigen::Vector3d xiAngDes = R_1_0 * controllerStates.desiredTwist.bottomRows<3>();

        controllerStates.sigma.topRows<3>() = xiLinDes + param.kSigma1 * posErr;
        controllerStates.sigma.bottomRows<3>() = xiAngDes + param.kSigma2 * epsilonErr;
        debugger.addEntry("sigma/beta", controllerStates.sigma);

        // Requirement: des twist in world frame, actual twist in base frame
        const Eigen::Vector3d posErrDot = xiLinDes - xiAbs.topRows<3>();
        const Eigen::Vector3d omegaErr = xiAngDes - xiAbs.bottomRows<3>();
        const Eigen::Vector3d epsilonErrDot_2 = etaErr*omegaErr + epsilonErr.cross(omegaErr);   // _2 because the factor 0.5 is applied later
#ifndef NDEBUG
        debugger.addEntry("d/dt position error", posErrDot);
        debugger.addEntry("d/dt epsilon error", 0.5*epsilonErrDot_2);
#endif  // NDEBUG

        const Eigen::Vector3d xiLinDesDot = xiLinDes.cross(xiAbs.bottomRows<3>());
        const Eigen::Vector3d xiAngDesDot = xiAngDes.cross(xiAbs.bottomRows<3>());

        controllerStates.sigmaDot.topRows<3>() = xiLinDesDot + param.kSigma1 * posErrDot;
        controllerStates.sigmaDot.bottomRows<3>() = xiAngDesDot + 0.5 * param.kSigma2 * epsilonErrDot_2;
        debugger.addEntry("d/dt sigma/beta", controllerStates.sigmaDot);

#ifdef IGNORE_Z_ERROR
        controllerStates.sigma(2) = controllerStates.sigmaDot(2) = 0.0;
#endif  // IGNORE_Z_ERROR
    }


public:
    BaseVehicleController(const std::string& name)
    : VehicleController(name, 0)
    , fixed(false)
    {
        VehicleController* base = static_cast<VehicleController*>(this);
        base->dynamicReconfigureManager->initCallback(base);
        ROS_INFO("Constructed base vehicle '%s'", name.c_str());
    }

    ~BaseVehicleController()
    {}


    bool fix(bool _fixed)
    {
        if (_fixed && !fixed) ROS_WARN("Fixing Base vehicle '%s'. The vehicle should be physically fixed!", nh->getNamespace().c_str());
        fixed = _fixed;
        return true;
    }

    /**
     * @brief update state vector
     * 
     * @param newState vehicle pose in map coordinates and twist in base coordinates
     */
    void updateVehicleState(const std::shared_ptr<StateProvider> newState)
    {
        if (newState == NULL) throw auto_print_error("New state is not initialized!");

        if (fixed) {
            xiAbs.fill(0);
            return;
        }

        poseAbs = newState->getPose<Eigen::Vector7d>();
        xiAbs = newState->getTwist<Eigen::Vector6d>();
        debugger.addEntry("abs pose", poseAbs);
        debugger.addEntry("abs vel", xiAbs);
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
        tau = vehicleModel.calcWrenches(xiAbs, controllerStates.sigma, controllerStates.sigmaDot, &debugger);
        debugger.addEntry("tau", tau);
    }

    void calcB(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 4>> B,
               const std::vector<int>& idxList) const
    {
        if (fixed) {
            B.topRows<6>().fill(0.0);
        } else {
            B.topRows<6>() = thrusterModel.Psi;
        }
    }

    /**
     * @brief calculate off-diagonal block matrices of motor force map
     * 
     * @param B 
     * @param X 
     */
    void calcOffDiagB(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 4>> B,
                      Eigen::Matrix<double, 6, 4>& X,
                      const std::vector<int>& idxList) const
    {
        B.topRows<6>() = X;
    }

    void calcEta(Eigen::Ref<Eigen::VectorXd> eta, const int idx)
    {
        if (fixed) {
            eta.topRows<6>().fill(0);
            return;
        }

        const Eigen::Vector6d s = controllerStates.sigma - xiAbs;
        debugger.addEntry("s", s);
        eta.topRows<6>() = tau + param.kP * s + param.kSat * s / std::max(s.norm(), param.lim);
        debugger.addEntry("eta", eta.topRows<6>());
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