#ifndef CHILDVEHICLECONTROLLER_H
#define CHILDVEHICLECONTROLLER_H


#include <memory>
#include "VehicleController.h"


template<typename JointModel>
class ChildVehicleController : public VehicleController
{
private:
    const std::shared_ptr<VehicleController> parent;

    JointModel jointModel;

    struct ControllerParam {
        double kSigma;
        double kP;
        double kSat;
        double lim;
        double maxAngularError;
    } param;

    struct ControllerStates {
        typename JointModel::JointVector thetaDes;
        typename JointModel::JointVector zetaDes;
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
        param.lim = config.lim;
        param.maxAngularError = config.maxAngularError;
        ROS_INFO("Updated child controller parameters for '%s'", nh->getNamespace().c_str());
    }


public:
    ChildVehicleController(const std::shared_ptr<VehicleController> ptr, const std::string& name, const int id)
    : VehicleController(name, id)
    , parent(ptr)
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

        controllerStates.thetaDes = desiredState->getPose<typename JointModel::JointVector>();
        controllerStates.zetaDes = desiredState->getTwist<typename JointModel::JointVector>();
        jointModel.enforceBounds(controllerStates.thetaDes, controllerStates.zetaDes);
        debugger.addEntry("bounded desired pose", controllerStates.thetaDes);
        debugger.addEntry("bounded desired twist", controllerStates.zetaDes);
        controllerStates.sigma = jointModel.mapDerivative(controllerStates.zetaDes) + param.kSigma * limitError(controllerStates.thetaDes - jointModel.theta, param.maxAngularError);
        controllerStates.sigmaDot = param.kSigma * jointModel.mapDerivative(controllerStates.zetaDes - jointModel.zeta);
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

        tau = vehicleModel.calcWrenches(xiAbs, controllerStates.beta, controllerStates.betaDot, &debugger);
        debugger.addEntry("tau", tau);
    }

    void calcEta(Eigen::Ref<Eigen::VectorXd> eta, const int idx)
    {
        parent->setJointWrenches(jointModel.transposedTransform(tau));
        debugger.addEntry("transformed tau", Eigen::Vector6d(jointModel.transposedTransform(tau)));
        const typename JointModel::JointVector s = controllerStates.sigma - jointModel.zeta;
        debugger.addEntry("s", s);
        if constexpr (std::is_same<typename JointModel::JointVector, double>::value) {
            eta(idx) = jointModel.Phi.transpose() * tau + param.kP * s + param.kSat * s / std::max(std::abs(s), param.lim);
            debugger.addEntry("eta", eta(idx));
        } else {
            eta.middleRows<JointModel::DOF>(idx) = jointModel.Phi.transpose() * tau + param.kP * s + param.kSat * s / std::max(s.norm(), param.lim);
            debugger.addEntry("eta", eta.middleRows<JointModel::DOF>(idx));
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
        B.middleRows<JointModel::DOF>(idxList[ID]) = jointModel.Phi.transpose() * X;
        X = jointModel.transposedTransform(X).eval();
        parent->calcOffDiagB(B, X, idxList);
    }

    // first step of calculating B recursively
    void calcB(Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 4>> B,
               const std::vector<int>& idxList) const
    {
        assert(idxList.size() >= ID);
        B.middleRows<JointModel::DOF>(idxList[ID]) = jointModel.Phi.transpose() * thrusterModel.Psi;
        Eigen::Matrix<double, 6, 4> X = jointModel.transposedTransform(thrusterModel.Psi);
        parent->calcOffDiagB(B, X, idxList);
    }

    uint getDof() const
    {
        return JointModel::DOF;
    }
};


#endif  // CHILDVEHICLECONTROLLER_H