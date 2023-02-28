#ifndef DISCRETETARGETMODE_H
#define DISCRETETARGETMODE_H


#include "TargetMode.h"


class DiscreteMode : public TargetMode
{
protected:
    ros::Timer periodicTimer;
    boost::function<void()> f;

    void update(const ros::TimerEvent& e)
    {
        f();
    }


public:
    DiscreteMode(const double _deltaT, const double _duration, boost::function<void()> _f)
    : TargetMode(_duration)
    , periodicTimer(nh.createTimer(ros::Duration(_deltaT), &DiscreteMode::update, this, false, false))
    , f(_f)
    {}

    ~DiscreteMode() = default;


    void start() override
    {
        periodicTimer.start();
        stopTimer.start();
        f();
    }
};


#endif  // DISCRETETARGETMODE_H