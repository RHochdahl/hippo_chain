#ifndef JOINTSINEWAVETARGETMODE_H
#define JOINTSINEWAVETARGETMODE_H


#include "../ContinuousMode.h"


class JointSineWaveMode : public ContinuousMode
{
private:
    int amplitude;
    double frequency;
    boost::function<void(double, double)> setAngleFunction;

    void step(double time)
    {
        const double phase = frequency*time;
        const double angle = amplitude * std::sin(phase);
        const double omega = amplitude*frequency * std::cos(phase);
        setAngleFunction(angle, omega);
    }


public:
    JointSineWaveMode(const int _amplitude, const double _period, const double _duration, boost::function<void(int, double)> _setAngleFunction)
    : ContinuousMode(_duration)
    , amplitude(_amplitude)
    , frequency(2*M_PI/_period)
    , setAngleFunction(_setAngleFunction)
    {
        ROS_INFO("Start Sine Wave!");
    }

    ~JointSineWaveMode() = default;
};



#endif  // JOINTSINEWAVETARGETMODE_H