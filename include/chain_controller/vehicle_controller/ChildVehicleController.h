#ifndef CHILDVEHICLECONTROLLER_H
#define CHILDVEHICLECONTROLLER_H


#include <memory>
#include "VehicleController.h"
#include "../joint_model/JointModel.h"
#include <hippo_chain/ChildControllerConfig.h>


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
    } param;

    struct ControllerStates {
        typename JointModel::JointVector thetaDes;
        typename JointModel::JointVector zetaDes;
        Eigen::Vector6d beta;
        Eigen::Vector6d betaDot;
        typename JointModel::JointVector sigma;
        typename JointModel::JointVector sigmaDot;
    } controllerStates;

    dynamic_reconfigure::Server<hippo_chain::ChildControllerConfig> server;
    dynamic_reconfigure::Server<hippo_chain::ChildControllerConfig>::CallbackType f;


    void updateControlParameters(hippo_chain::ChildControllerConfig &config, uint32_t level)
    {
        param.kSigma = config.kSigma;
        param.kP = config.kP;
        param.kSat = config.kSat;
        param.lim = config.lim;
        ROS_INFO("Updated controller parameters for '%s'", nh->getNamespace().c_str());
    }


public:
    ChildVehicleController(const std::shared_ptr<VehicleController> ptr, const std::string& name)
    : VehicleController(name)
    , parent(ptr)
    , jointModel(configProvider)
    , server(*nh)
    , f(boost::bind(&ChildVehicleController::updateControlParameters, this, _1, _2))
    {
        server.setCallback(f);
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
        jointModel.update(newState);
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

        controllerStates.thetaDes = desiredState->getPose<typename JointModel::JointVector>();
        controllerStates.zetaDes = desiredState->getTwist<typename JointModel::JointVector>();
        jointModel.enforceBounds(controllerStates.thetaDes, controllerStates.zetaDes);
        controllerStates.sigma = controllerStates.zetaDes + param.kSigma * (controllerStates.thetaDes - jointModel.theta);
        controllerStates.sigmaDot = param.kSigma * (controllerStates.zetaDes - jointModel.zeta);

        xiAbs = jointModel.transform(parent->getXiAbs()) + jointModel.xiRel;

        const Eigen::Vector6d betaRel = jointModel.mapVelocity(controllerStates.sigma);
        const Eigen::Vector6d betaRelDot = jointModel.mapAcceleration(controllerStates.sigmaDot, controllerStates.sigma);
        const Eigen::Vector6d betaTemp = jointModel.transform(parent->getBeta());
        controllerStates.beta = betaTemp + betaRel;
        controllerStates.betaDot = jointModel.transform(parent->getBetaDot()) + betaRelDot + shared::cross6(betaTemp, jointModel.xiRel);

        tau = vehicleModel.calcWrenches(xiAbs, controllerStates.beta, controllerStates.betaDot);
    }

    Eigen::VectorXd calcEta() const
    {
        parent->setJointWrenches(jointModel.inverseTransform(tau));
        const typename JointModel::JointVector s = controllerStates.sigma - jointModel.zeta;
        if constexpr (std::is_same<typename JointModel::JointVector, double>::value)
            return Eigen::Vector1d(jointModel.Phi.transpose() * tau + param.kP * s + param.kSat * s / std::max(std::abs(s), param.lim));
        else
            return jointModel.Phi.transpose() * tau + param.kP * s + param.kSat * s / std::max(s.norm(), param.lim);
    }

    /**
     * @brief calculate off-diagonal block matrices of motor force map
     * 
     * @param B 
     * @param X 
     */
    void calcOffDiagB(std::vector<std::pair<uint, Eigen::Matrix<double, Eigen::Dynamic, 4>>>& B, Eigen::Matrix<double, 6, 4>& X) const
    {
        B.push_back(std::make_pair(ID, jointModel.Phi.transpose() * X));
        X = jointModel.inverseTransform(X);
        parent->calcOffDiagB(B, X);
    }

    // first step of calculating B recursively
    void calcB(std::vector<std::pair<uint, Eigen::Matrix<double, Eigen::Dynamic, 4>>>& B) const
    {
        B.push_back(std::make_pair(ID, jointModel.Phi.transpose() * thrusterModel.Psi));
        Eigen::Matrix<double, 6, 4> X = jointModel.inverseTransform(thrusterModel.Psi);
        parent->calcOffDiagB(B, X);
    }

    uint getDof() const
    {
        return JointModel::DOF;
    }
};


#endif  // CHILDVEHICLECONTROLLER_H