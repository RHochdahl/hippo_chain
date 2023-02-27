#ifndef JOINTRAMPWAVETARGETMODE_H
#define JOINTRAMPWAVETARGETMODE_H


#include "ContinuousMode.h"


class JointRampWaveMode : public ContinuousMode
{
private:
    double amplitude;
    double frequency;
    double velocity;
    double startPoint;
    int direction;
    boost::function<void(double, double)> setFormFunction;

    void step(double time)
    {
        double phase = startPoint + direction*frequency*time;

        if (phase > 1.0) {
            startTime = startTime + ros::Duration((1.0-startPoint)/frequency);
            phase = 2.0 - phase;
            direction = -1;
            startPoint = 1.0;
        } else if (phase < -1.0) {
            startTime = startTime + ros::Duration((1.0+startPoint)/frequency);
            phase = -2.0 - phase;
            direction = 1;
            startPoint = -1.0;
        }

        setFormFunction(amplitude*phase, velocity);
    }
    
    void neutral()
    {
        setFormFunction(0.0, 0.0);
    }


public:
    JointRampWaveMode(const double _amplitude, const double _period, const double _duration, const double _dT, const bool _setVelocity, boost::function<void(double, double)> _setFormFunction)
    : ContinuousMode(_dT, _duration)
    , amplitude(_amplitude)
    , frequency(1.0/_period)
    , velocity(_setVelocity ? (4*_amplitude/_period) : 0.0)
    , startPoint(0.0)
    , direction(1)
    , setFormFunction(_setFormFunction)
    {
        ROS_INFO("Start Ramp Wave!");
    }

    ~JointRampWaveMode() = default;
};



#endif  // JOINTRAMPWAVETARGETMODE_H