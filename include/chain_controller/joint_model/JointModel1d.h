#ifndef JOINTMODEL1D_H
#define JOINTMODEL1D_H


#include "JointModel.h"
#include "bounds/BoundsDouble.h"

// every class inheriting from this must init Phi and Theta
class JointModel1d : public JointModel<1>
{
protected:
    virtual void calcA() = 0;
    virtual void calcPhi() = 0;
    virtual void calcTheta() = 0;

public:
    void update(const std::shared_ptr<StateProvider> newState)
    {
        theta = newState->getPose<JointVector>();
        zeta = newState->getTwist<JointVector>();
        calcA();
        xiRel = mapVelocity(zeta);
        if (!bounds->checkBounds(theta, zeta)) ROS_WARN_THROTTLE(5.0, "Joint out of bounds!");
    }

    Eigen::Vector6d mapVelocity(const double velocity) const
    {
        return Phi * velocity;
    }

    Eigen::Vector6d mapAcceleration(const double acceleration) const
    {
        return Phi * acceleration;
    }

    Eigen::Vector6d mapAcceleration(const double acceleration, const double velocity) const
    {
        return mapAcceleration(acceleration);
    }

    Eigen::Vector6d mapAcceleration(const Eigen::Matrix<double, 1, 1>& acceleration) const
    {
        return mapAcceleration(0);
    }

    Eigen::Vector6d mapAcceleration(const Eigen::Matrix<double, 1, 1>& acceleration, const Eigen::Matrix<double, 1, 1>& velocity) const
    {
        return mapAcceleration(acceleration(0));
    }
};


#endif  // JOINTMODEL1D_H