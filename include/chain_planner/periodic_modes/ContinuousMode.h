#ifndef CONTINUOUSTARGETMODE_H
#define CONTINUOUSTARGETMODE_H


#include "PeriodicMode.h"


class ContinuousMode : public PeriodicMode
{
protected:
    ros::Duration dT;
    ros::Time startTime;
    bool initialized;


    void callback()
    {
        if (initialized) {
            step((ros::Time::now() - startTime).toSec());
        } else {
            startTime = ros::Time::now();
            initialized = true;
            step(0.0);
        }
    }

    virtual void step(const double time) = 0;
    virtual void neutral() = 0;


public:
    ContinuousMode(const double deltaT, const double duration)
    : PeriodicMode(deltaT, duration, boost::bind(&ContinuousMode::callback, this))
    , dT(deltaT)
    , startTime()
    , initialized(false)
    {}


    ~ContinuousMode() = default;
};


#endif  // CONTINUOUSTARGETMODE_H