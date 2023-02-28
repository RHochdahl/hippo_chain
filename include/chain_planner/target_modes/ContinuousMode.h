#ifndef CONTINUOUSTARGETMODE_H
#define CONTINUOUSTARGETMODE_H


#include "TargetMode.h"


class ContinuousMode : public TargetMode
{
protected:
    ros::Time startTime;
    bool initialized;

    virtual void step(const double time) = 0;


public:
    ContinuousMode(const double _duration)
    : TargetMode(_duration)
    , startTime()
    , initialized(false)
    {}

    ~ContinuousMode() = default;


    bool set() override
    {
        if (initialized) {
            step((ros::Time::now() - startTime).toSec());
        } else {
            startTime = ros::Time::now();
            initialized = true;
            step(0.0);
        }
        return finished;
    }
};


#endif  // CONTINUOUSTARGETMODE_H