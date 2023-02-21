#ifndef JOINTMODEL_H
#define JOINTMODEL_H


#include <hippo_chain/include/common/typedefs.h>
#include <hippo_chain/include/common/ConfigProvider.h>
#include <hippo_chain/include/chain_controller/state/StateProvider.h>
#include "bounds/Bounds.h"


template<std::size_t dof>
class JointModel
{
protected:
    virtual void calcA() = 0;
    virtual void calcPhi() = 0;
    virtual void calcTheta() = 0;
    virtual void calcT() = 0;

public:
    static constexpr int DOF = dof;
    typedef typename std::conditional<DOF == 1, double, Eigen::Matrix<double, DOF, 1>>::type JointVector;
    typedef typename std::conditional<DOF == 1, double, Eigen::Matrix<double, DOF, DOF>>::type JointMatrix;

    JointVector theta;                      // joint coordinates
    JointVector zeta;                       // joint velocities
    JointMatrix T;                          // matrix to get derivatives of joint coordinates from joint velocities
    Eigen::Matrix6d A;                      // transformation matrix to parent frame
    Eigen::Matrix<double, 6, dof> Phi;      // jacobian of joint velocities to relative velocities
    Eigen::Matrix<double, 6, dof> Theta;    // time derivative of Phi
    Eigen::Vector6d xiRel;                  // relative velocity to parent (in child frame)

    std::unique_ptr<Bounds<JointVector>> bounds;


    void enforceBounds(JointVector& desPose, JointVector& desTwist) const
    {
        if (!bounds->enforceBounds(desPose, desTwist)) ROS_WARN_THROTTLE(5.0, "Joint target out of bounds!");
    }


    template<typename Derived>
    auto mapDerivative(const Eigen::MatrixBase<Derived>& velocity) const
    {
        return T * velocity;
    }

    template<typename Derived>
    auto mapVelocity(const Eigen::MatrixBase<Derived>& velocity) const
    {
        return Phi * velocity;
    }

    template<typename Derived1,
             typename Derived2>
    auto mapAcceleration(const Eigen::MatrixBase<Derived1>& acceleration,
                         const Eigen::MatrixBase<Derived2>& velocity) const
    {
        return Phi * acceleration + Theta * velocity;
    }

    template<typename Derived>
    auto transform(const Eigen::MatrixBase<Derived>& input) const
    {
        return A * input;
    }

    template<typename Derived>
    auto transposedTransform(const Eigen::MatrixBase<Derived>& input) const
    {
        return A.transpose() * input;
    }


    void update(const std::shared_ptr<StateProvider> newState)
    {
        theta = newState->getPose<JointVector>();
        zeta = newState->getTwist<JointVector>();
        calcA();
        calcPhi();
        calcTheta();
        calcT();
        xiRel = mapVelocity(zeta);
        if (!bounds->checkBounds(theta, zeta)) ROS_WARN_THROTTLE(5.0, "Joint out of bounds!");
    }
};


#endif  // JOINTMODEL_H