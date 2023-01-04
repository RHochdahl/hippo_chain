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
    // must be above update()
    auto mapVelocity(const double velocity) const
    {
        return Phi * velocity;
    }

    void update(const std::shared_ptr<StateProvider> newState)
    {
        theta = newState->getPose<JointVector>();
        zeta = newState->getTwist<JointVector>();
        calcA();
        xiRel = mapVelocity(zeta);
        if (!bounds->checkBounds(theta, zeta)) ROS_WARN_THROTTLE(5.0, "Joint out of bounds!");
    }


    auto mapAcceleration(const double acceleration) const
    {
        return Phi * acceleration;
    }

    auto mapAcceleration(const double acceleration, const double velocity) const
    {
        return mapAcceleration(acceleration);
    }
};


#endif  // JOINTMODEL1D_H