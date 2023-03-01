#ifndef JOINTSETPOINTMODE_H
#define JOINTSETPOINTMODE_H


#include "../DiscreteMode.h"


class JointSetpointMode : public DiscreteMode
{
private:
    double angle;
    double amplitude;
    double stepSize;
    int direction;
    boost::function<void(double)> setAngleFunction;

    void callback()
    {
        angle += direction * stepSize;

        if (angle > amplitude) {
            angle = 2*amplitude - angle;
            direction = -1;
        } else if (angle < -amplitude) {
            angle = -2*amplitude - angle;
            direction = 1;
        }

        setAngleFunction(angle);
    }


public:
    JointSetpointMode(const double _amplitude, const double _stepSize, const double _dT, const double _duration, boost::function<void(double)> _setAngleFunction)
    : DiscreteMode(_dT, _duration, boost::bind(&JointSetpointMode::callback, this))
    , angle(0)
    , amplitude(_amplitude)
    , stepSize(_stepSize)
    , direction(1)
    , setAngleFunction(_setAngleFunction)
    {
        ROS_INFO("Start setpoint mode!");
    }

    ~JointSetpointMode() = default;
};


#endif  // JOINTSETPOINTMODE_H