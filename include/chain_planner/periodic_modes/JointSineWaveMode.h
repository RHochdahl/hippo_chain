#ifndef JOINTSINEWAVETARGETMODE_H
#define JOINTSINEWAVETARGETMODE_H


#include "ContinuousMode.h"


class JointSineWaveMode : public ContinuousMode
{
private:
    double amplitude;
    double frequency;
    boost::function<void(double, double)> setFormFunction;

    void step(double time)
    {
        const double phase = frequency*time;
        const double form = amplitude * std::sin(phase);
        const double rate = amplitude*frequency * std::cos(phase);
        setFormFunction(form, rate);
    }

    void neutral()
    {
        setFormFunction(0.0, 0.0);
    }


public:
    JointSineWaveMode(const double _amplitude, const double _period, const double _duration, boost::function<void(double, double)> _setFormFunction)
    : ContinuousMode(0.05, _duration)
    , amplitude(_amplitude)
    , frequency(2*M_PI/_period)
    , setFormFunction(_setFormFunction)
    {
        ROS_INFO("Start Sine Wave!");
    }

    ~JointSineWaveMode() = default;
};



#endif  // JOINTSINEWAVETARGETMODE_H