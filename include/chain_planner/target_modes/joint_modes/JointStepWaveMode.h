#ifndef JOINTSTEPWAVETARGETMODE_H
#define JOINTSTEPWAVETARGETMODE_H


#include "../DiscreteMode.h"


class JointStepWaveMode : public DiscreteMode
{
private:
    double angle;
    boost::function<void(double)> setAngleFunction;

    void callback()
    {
        angle *= -1;
        setAngleFunction(angle);
    }


public:
    JointStepWaveMode(const double _amplitude, const double _period, const double _duration, boost::function<void(double)> _setAngleFunction)
    : DiscreteMode(0.5*_period, _duration, boost::bind(&JointStepWaveMode::callback, this))
    , angle(_amplitude)
    , setAngleFunction(_setAngleFunction)
    {
        ROS_INFO("Start Step Wave!");
    }

    ~JointStepWaveMode() = default;
};

#endif  // JOINTSTEPWAVETARGETMODE_H