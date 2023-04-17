#ifndef CHILDVEHICLECONTROLLER_H
#define CHILDVEHICLECONTROLLER_H


#include <memory>
#include "VehicleController.h"
#include <hippo_chain/Error.h>
#include <hippo_chain/include/common/sharedAlgorithms.hpp>


template<typename JointModel>
class ChildVehicleController : public VehicleController
{
private:
    const std::shared_ptr<VehicleController> parent;

    ros::Publisher error_pub;

    JointModel jointModel;

    struct ControllerParam {
        double kSigma;
        double kP;
        double kSat;
        double rho;
        double maxAngularError;
        double lsqWeight;
    } param;

    struct ControllerStates {
        typename JointModel::JointVector thetaDes;
        typename JointModel::JointVector zetaDes;
        typename JointModel::JointVector zetaDotDes;
        Eigen::Vector6d beta;
        Eigen::Vector6d betaDot;
        typename JointModel::JointVector sigma;
        typename JointModel::JointVector sigmaDot;
    } controllerStates;


    void updateControlParameters(const hippo_chain::VehicleControllerConfig& config, uint32_t level)
    {
        if (!(level & DynamicReconfigureLevels::CHILD)) return;
        param.kSigma = config.kSigma;
        param.kP = config.kP;
        param.kSat = config.kSat;
        param.rho = config.rho;
        param.maxAngularError = config.maxAngularError;
        param.lsqWeight = config.lsqWeight;
        ROS_INFO("Updated child controller parameters for '%s'", nh->getNamespace().c_str());
    }


public:
    ChildVehicleController(const std::shared_ptr<VehicleController> ptr, const std::string& name, const int id)
    : VehicleController(name, id)
    , parent(ptr)
    , error_pub(nh->advertise<hippo_chain::Error>("error", 1))
    , jointModel(configProvider)
    {
        assert(ID > 0);
        VehicleController* base = static_cast<VehicleController*>(this);
        base->dynamicReconfigureManager->initCallback(base);
        ROS_INFO("Constructed child vehicle '%s'", name.c_str());
    }

    ~ChildVehicleController()
    {}

    /**
     * @brief update Phi, Theta, A, theta, zeta
     * 
     * @param newState 
     */
    void updateVehicleState(const std::shared_ptr<StateProvider> newState)
    {
        if (newState == NULL) throw auto_print_error("New state is not initialized!");
        debugger.addEntry("pose", newState->pose.data(), newState->pose.size());
        debugger.addEntry("twist", newState->twist.data(), newState->twist.size());
        jointModel.update(newState);
        debugger.addEntry("A", jointModel.A);
        debugger.addEntry("Phi", jointModel.Phi);
        debugger.addEntry("Theta", jointModel.Theta);
        debugger.addEntry("rel vel", jointModel.xiRel);
    }

    const Eigen::Vector6d& getBeta() const
    {
        return controllerStates.beta;
    }

    const Eigen::Vector6d& getBetaDot() const
    {
        return controllerStates.betaDot;
    }

    void calcDecoupledWrenches(const std::shared_ptr<StateProvider> desiredState)
    {
        if (desiredState == NULL) throw auto_print_error("Desired state is not initialized!");

        debugger.addEntry("desired pose", desiredState->pose.data(), desiredState->pose.size());
        debugger.addEntry("desired twist", desiredState->twist.data(), desiredState->twist.size());

        controllerStates.thetaDes = desiredState->get_pose<typename JointModel::JointVector>();
        controllerStates.zetaDes = desiredState->get_twist<typename JointModel::JointVector>();
        controllerStates.zetaDotDes = desiredState->get_accel<typename JointModel::JointVector>();
        jointModel.enforceBounds(controllerStates.thetaDes, controllerStates.zetaDes);
        debugger.addEntry("bounded desired pose", controllerStates.thetaDes);
        debugger.addEntry("bounded desired twist", controllerStates.zetaDes);
        debugger.addEntry("bounded desired accel", controllerStates.zetaDotDes);
        const typename JointModel::JointVector angleError = limitError(typename JointModel::JointVector(controllerStates.thetaDes - jointModel.theta), param.maxAngularError);
        const typename JointModel::JointVector twistError = controllerStates.zetaDes - jointModel.zeta;
        controllerStates.sigma = controllerStates.zetaDes + param.kSigma * angleError;

        if (errorPub.getNumSubscribers()) {
            hippo_chain::Error errorMsg;
            errorMsg.header.stamp       = ros::Time::now();
            errorMsg.state.pose         = shared::toArray<7>(jointModel.theta);
            errorMsg.state.twist        = shared::toArray<6>(jointModel.zeta);
            errorMsg.des_state.pose     = shared::toArray<7>(controllerStates.thetaDes);
            errorMsg.des_state.twist    = shared::toArray<6>(controllerStates.zetaDes);
            errorMsg.des_state.accel    = shared::toArray<6>(controllerStates.zetaDotDes);
            errorMsg.error.pose         = shared::toArray<7>(angleError);
            errorMsg.error.twist        = shared::toArray<6>(twistError);
            errorMsg.sigma.pose         = shared::toArray<7>(controllerStates.sigma);
            errorMsg.sigma.twist        = shared::toArray<6>(controllerStates.sigmaDot);
            errorPub.publish(errorMsg);
        }

        controllerStates.sigmaDot = controllerStates.zetaDotDes + param.kSigma * twistError;
        debugger.addEntry("sigma", controllerStates.sigma);
        debugger.addEntry("d/dt sigma", controllerStates.sigmaDot);

        xiAbs = jointModel.transform(parent->getXiAbs()) + jointModel.xiRel;
        debugger.addEntry("abs vel parent", jointModel.transform(parent->getXiAbs()));
        debugger.addEntry("abs vel", xiAbs);

        const Eigen::Vector6d betaTemp = jointModel.transform(parent->getBeta());
        controllerStates.beta = jointModel.mapVelocity(controllerStates.sigma) + betaTemp;
        controllerStates.betaDot = jointModel.transform(parent->getBetaDot())
                                 + jointModel.mapAcceleration(controllerStates.sigmaDot, controllerStates.sigma)
                                 + shared::cross6(betaTemp, jointModel.xiRel);
        debugger.addEntry("beta parent", betaTemp);
        debugger.addEntry("beta", controllerStates.beta);
        debugger.addEntry("d/dt beta", controllerStates.betaDot);

        vehicleModel.setVelocity(xiAbs);
        tau = vehicleModel.calcWrenches(controllerStates.beta, controllerStates.betaDot, &debugger);
        debugger.addEntry("tau", tau);
    }

    void calcEta(Eigen::Ref<Eigen::VectorXd> eta, const int idx)
    {
        parent->setJointWrenches(jointModel.transposedTransform(tau));
        debugger.addEntry("transformed tau", Eigen::Vector6d(jointModel.transposedTransform(tau)));
        const typename JointModel::JointVector s = controllerStates.sigma - jointModel.zeta;
        debugger.addEntry("s", s);
        if constexpr (std::is_same<typename JointModel::JointVector, double>::value) {
            eta(idx) = jointModel.Phi.transpose() * tau + param.kP * s + param.kSat * shared::sat(s, param.rho);
            debugger.addEntry("eta", eta(idx));
            eta(idx) *= param.lsqWeight;
        } else {
            eta.middleRows<JointModel::DOF>(idx) = jointModel.Phi.transpose() * tau + param.kP * s + param.kSat * shared::sat(s, param.rho);
            debugger.addEntry("eta", eta.middleRows<JointModel::DOF>(idx));
            eta.middleRows<JointModel::DOF>(idx) *= param.lsqWeight;
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
        B.middleRows<JointModel::DOF>(idxList[ID]) = param.lsqWeight * jointModel.Phi.transpose() * X;
        X = jointModel.transposedTransform(X).eval();
        parent->calcOffDiagB(B, X, idxList);
    }

    // first step of calculating B recursively
    void calcB(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 4>> B,
               const std::vector<int>& idxList) const
    {
        assert(idxList.size() >= ID);
        B.middleRows<JointModel::DOF>(idxList[ID]) = param.lsqWeight * jointModel.Phi.transpose() * thrusterModel.Psi;
        Eigen::Matrix<double, 6, 4> X = jointModel.transposedTransform(thrusterModel.Psi);
        parent->calcOffDiagB(B, X, idxList);
    }

    uint getDof() const
    {
        return JointModel::DOF;
    }
};


#endif  // CHILDVEHICLECONTROLLER_H