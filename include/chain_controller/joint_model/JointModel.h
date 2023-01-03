#ifndef JOINTMODEL_H
#define JOINTMODEL_H


#include "../../utils/typedefs.h"
#include "../../utils/ConfigProvider.h"
#include "../state/StateProvider.h"
#include "bounds/Bounds.h"


template<std::size_t dof>
class JointModel
{
protected:
    virtual void calcA() = 0;
    virtual void calcPhi() = 0;
    virtual void calcTheta() = 0;

public:
    static constexpr int DOF = dof;
    typedef typename std::conditional<DOF == 1, double, Eigen::Matrix<double, DOF, 1>>::type JointVector;

    JointVector theta;                      // joint coordinates
    JointVector zeta;                       // joint velocities
    Eigen::Matrix6d A;                      // transformation matrix to parent frame
    Eigen::Matrix<double, 6, dof> Phi;      // jacobian of joint velocities to relative velocities
    Eigen::Matrix<double, 6, dof> Theta;    // time derivative of Phi
    Eigen::Vector6d xiRel;                  // relative velocity to parent (in child frame)

    std::unique_ptr<Bounds<JointVector>> bounds;


    void update(const std::shared_ptr<StateProvider> newState)
    {
        theta = newState->getPose<JointVector>();
        zeta = newState->getTwist<JointVector>();
        calcA();
        calcPhi();
        calcTheta();
        xiRel = mapVelocity(zeta);
        if (!bounds->checkBounds(theta, zeta)) ROS_WARN_THROTTLE(5.0, "Joint out of bounds!");
    }

    void enforceBounds(JointVector& desPose, JointVector& desTwist) const
    {
        if (!bounds->enforceBounds(desPose, desTwist)) ROS_WARN_THROTTLE(5.0, "Joint target out of bounds!");
    }

    template<typename Derived>
    Eigen::Vector6d mapVelocity(const Eigen::MatrixBase<Derived>& velocity) const
    {
        return Phi * velocity;
    }

    template<typename Derived1,
             typename Derived2>
    Eigen::Vector6d mapAcceleration(const Eigen::MatrixBase<Derived1>& acceleration,
                                    const Eigen::MatrixBase<Derived2>& velocity) const
    {
        return Phi * acceleration + Theta * velocity;
    }

    template<typename T>
    T transform(const T& input) const
    {
        return A * input;
    }

    template<typename T>
    T transposedTransform(const T& input) const
    {
        return A.transpose() * input;
    }
};


#endif  // JOINTMODEL_H