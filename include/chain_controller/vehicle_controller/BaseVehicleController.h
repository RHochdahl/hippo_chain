#ifndef BASEVEHICLECONTROLLER_H
#define BASEVEHICLECONTROLLER_H


#include "VehicleController.h"
#include <hippo_chain/FixBase.h>


// #define IGNORE_Z_ERROR


class BaseVehicleController : public VehicleController
{
private:
    struct ControllerParam {
        double kSigma1;
        double kSigma2;
        double kP;
        double kSat;
        double rho;
        double maxPositionError;
        double maxQuaternionError;
    } param;

    struct ControllerStates {
        Eigen::Vector7d desiredPose;
        Eigen::Vector6d desiredTwist;
        Eigen::Vector6d desiredAccel;
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
        param.rho = config.rho;
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

        controllerStates.desiredPose = desiredState->get_pose<Eigen::Vector7d>();
        controllerStates.desiredTwist = desiredState->get_twist<Eigen::Vector6d>();
        controllerStates.desiredAccel = desiredState->get_accel<Eigen::Vector6d>();
        debugger.addEntry("desired pose", desiredState->pose.data(), desiredState->pose.size());
        debugger.addEntry("desired twist", desiredState->twist.data(), desiredState->twist.size());
        debugger.addEntry("desired accel", desiredState->accel.data(), desiredState->accel.size());

        const Eigen::Quaterniond quatDes(controllerStates.desiredPose[3],
                                         controllerStates.desiredPose[4],
                                         controllerStates.desiredPose[5],
                                         controllerStates.desiredPose[6]);
        const Eigen::Quaterniond quatActInv(poseAbs[3],
                                            -poseAbs[4],
                                            -poseAbs[5],
                                            -poseAbs[6]);

        if (!shared::isUnitQuaternion(quatDes)) throw quaternion_error(quatDes);
        if (!shared::isUnitQuaternion(quatActInv)) throw quaternion_error(quatActInv);

        debugger.addEntry("desired orientation", quatDes.coeffs());

        const Eigen::Quaterniond quatErr = quatActInv * quatDes;
        debugger.addEntry("quat error", quatErr.coeffs().data(), 4U);

        // rotation matrix R^1_0 or S_K'K to rotate vector from world to base frame
        const Eigen::Matrix3d R_1_0 = quatActInv.toRotationMatrix();
        debugger.addEntry("R_1_0", R_1_0);
        const Eigen::Matrix3d R_1_1des = quatErr.toRotationMatrix();
        debugger.addEntry("R_1_1des", R_1_1des);

        const Eigen::Vector3d posErr = limitError(Eigen::Vector3d(R_1_0 * (controllerStates.desiredPose.topRows<3>() - poseAbs.topRows<3>())), param.maxPositionError);
        const Eigen::Vector3d epsilonErr = limitError(Eigen::Vector3d(shared::sgn(quatErr.w()) * quatErr.vec()), param.maxQuaternionError);
        const double etaErr = std::sqrt(1 - epsilonErr.squaredNorm());
        debugger.addEntry("position error", posErr);
        debugger.addEntry("epsilon error", epsilonErr);
        debugger.addEntry("eta error", etaErr);

        // Requirement: des twist and actual twist in base frame
        const Eigen::Vector3d xiAngDes = R_1_1des * controllerStates.desiredTwist.bottomRows<3>();
        const Eigen::Vector3d xiLinDes = R_1_1des * controllerStates.desiredTwist.topRows<3>() + posErr.cross(xiAngDes);

        controllerStates.sigma.topRows<3>() = xiLinDes + param.kSigma1 * posErr;
        controllerStates.sigma.bottomRows<3>() = xiAngDes + param.kSigma2 * epsilonErr;
        debugger.addEntry("sigma/beta", controllerStates.sigma);

        const Eigen::Vector3d posErrDot = xiLinDes - xiAbs.topRows<3>();
        const Eigen::Vector3d omegaErr = xiAngDes - xiAbs.bottomRows<3>();
        const Eigen::Vector3d epsilonErrDot_2 = etaErr*omegaErr + omegaErr.cross(epsilonErr);   // _2 because the factor 0.5 is applied later
        debugger.addEntry("d/dt position error", posErrDot);
        debugger.addEntry("d/dt epsilon error", 0.5*epsilonErrDot_2);

        const Eigen::Vector3d temp = R_1_1des * controllerStates.desiredAccel.bottomRows<3>();
        const Eigen::Vector3d xiLinDesDot = omegaErr.cross(xiLinDes) + posErrDot.cross(xiAngDes) + R_1_1des * controllerStates.desiredAccel.topRows<3>() + posErr.cross(temp);
        const Eigen::Vector3d xiAngDesDot = omegaErr.cross(xiAngDes) + temp;

        controllerStates.sigmaDot.topRows<3>() = xiLinDesDot + param.kSigma1 * posErrDot;
        controllerStates.sigmaDot.bottomRows<3>() = xiAngDesDot + 0.5 * param.kSigma2 * epsilonErrDot_2;
        debugger.addEntry("d/dt sigma/beta", controllerStates.sigmaDot);

#ifdef IGNORE_Z_ERROR
        controllerStates.sigma(2) = controllerStates.sigmaDot(2) = 0.0;
#endif  // IGNORE_Z_ERROR

        if (errorPub.getNumSubscribers()) {
            hippo_chain::Error errorMsg;
            errorMsg.header.stamp = ros::Time::now();
            std::copy(poseAbs.data(), poseAbs.data()+7, errorMsg.state.pose.data());
            std::copy(xiAbs.data(), xiAbs.data()+6, errorMsg.state.twist.data());
            std::copy(controllerStates.desiredPose.data(), controllerStates.desiredPose.data()+7, errorMsg.des_state.pose.data());
            std::copy(controllerStates.desiredTwist.data(), controllerStates.desiredTwist.data()+6, errorMsg.des_state.twist.data());
            std::copy(posErr.data(), posErr.data()+3, errorMsg.error.pose.data());
            std::copy(quatErr.coeffs().data(), quatErr.coeffs().data()+4, errorMsg.error.pose.data()+4);
            std::copy(posErrDot.data(), posErrDot.data()+3, errorMsg.error.twist.data());
            std::copy(omegaErr.data(), omegaErr.data()+3, errorMsg.error.twist.data()+3);
            std::copy(controllerStates.desiredAccel.data(), controllerStates.desiredAccel.data()+6, errorMsg.des_state.accel.data());
            std::copy(controllerStates.sigma.data(), controllerStates.sigma.data()+6, errorMsg.sigma.pose.data());
            std::copy(controllerStates.sigmaDot.data(), controllerStates.sigmaDot.data()+6, errorMsg.sigma.twist.data());
            errorPub.publish(errorMsg);
        }
    }


public:
    BaseVehicleController(const std::string& name)
    : VehicleController(name, 0)
    , fixed(configProvider->getValueWithDefault("/fix_base", false))
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
        return fixed;
    }

    bool isFixed()
    {
        return fixed;
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

        poseAbs = newState->get_pose<Eigen::Vector7d>();
        xiAbs = newState->get_twist<Eigen::Vector6d>();
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
        vehicleModel.setVelocity(xiAbs);
        tau = vehicleModel.calcWrenches(controllerStates.sigma, controllerStates.sigmaDot, &debugger);
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
        if (fixed) {
            B.topRows<6>().fill(0.0);
        } else {
            B.topRows<6>() = X;
        }
    }

    void calcEta(Eigen::Ref<Eigen::VectorXd> eta, const int idx)
    {
        if (fixed) {
            eta.topRows<6>().fill(0);
            return;
        }

        const Eigen::Vector6d s = controllerStates.sigma - xiAbs;
        debugger.addEntry("s", s);
        eta.topRows<6>() = tau + param.kP * s + param.kSat * shared::sat(s, param.rho);
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