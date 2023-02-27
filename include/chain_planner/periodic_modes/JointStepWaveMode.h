#ifndef JOINTSTEPWAVETARGETMODE_H
#define JOINTSTEPWAVETARGETMODE_H


#include "PeriodicMode.h"


class JointStepWaveMode : public PeriodicMode
{
private:
    double form;
    boost::function<void(double)> setFormFunction;

    void callback()
    {
        form *= -1;
        setFormFunction(form);
    }

    void neutral()
    {
        setFormFunction(0.0);
    }


public:
    JointStepWaveMode(const double amplitude, const double period, const double duration, boost::function<void(double)> _setFormFunction)
    : PeriodicMode(0.5*period, duration, boost::bind(&JointStepWaveMode::callback, this))
    , form(amplitude)
    , setFormFunction(_setFormFunction)
    {
        ROS_INFO("Start Step Wave!");
    }

    ~JointStepWaveMode() = default;
};

#endif  // JOINTSTEPWAVETARGETMODE_H