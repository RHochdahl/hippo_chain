#ifndef JOINTRAMPWAVETARGETMODE_H
#define JOINTRAMPWAVETARGETMODE_H


#include "../ContinuousMode.h"


class JointRampWaveMode : public ContinuousMode
{
private:
    int amplitude;
    double frequency;
    double velocity;
    int startPoint;
    int direction;
    boost::function<void(double, double)> setAngleFunction;

    void step(double time)
    {
        double phase = startPoint + direction*frequency*time;

        if (phase > 1.0) {
            startTime = startTime + ros::Duration((1.0-startPoint)/frequency);
            phase = 2.0 - phase;
            direction = -1;
            startPoint = 1;
        } else if (phase < -1.0) {
            startTime = startTime + ros::Duration((1.0+startPoint)/frequency);
            phase = -2.0 - phase;
            direction = 1;
            startPoint = -1;
        }

        setAngleFunction(amplitude*phase, velocity);
    }


public:
    JointRampWaveMode(const int _amplitude, const double _period, const double _duration, boost::function<void(double, double)> _setAngleFunction)
    : ContinuousMode(_duration)
    , amplitude(_amplitude)
    , frequency(1.0/_period)
    , velocity(4*_amplitude/_period)
    , startPoint(0)
    , direction(1)
    , setAngleFunction(_setAngleFunction)
    {
        ROS_INFO("Start Ramp Wave!");
    }

    ~JointRampWaveMode() = default;
};



#endif  // JOINTRAMPWAVETARGETMODE_H