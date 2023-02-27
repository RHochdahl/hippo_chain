#ifndef PERIODICTARGETMODE_H
#define PERIODICTARGETMODE_H


#include <ros/ros.h>


class PeriodicMode
{
protected:
    ros::NodeHandle nh;
    ros::Timer periodicTimer;
    ros::Timer startTimer;
    ros::Timer stopTimer;
    boost::function<void()> f;

    void update(const ros::TimerEvent& e)
    {
        f();
    }

    void stop(const ros::TimerEvent& e)
    {
        stop();
    }

    virtual void neutral() = 0;


public:
    PeriodicMode(const double deltaT, const double duration, boost::function<void()> _f)
    : nh()
    , periodicTimer(nh.createTimer(ros::Duration(deltaT), &PeriodicMode::update, this, false, false))
    , stopTimer(nh.createTimer(ros::Duration(duration), &PeriodicMode::stop, this, false, false))
    , f(_f)
    {}


    void start()
    {
        periodicTimer.start();
        stopTimer.start();
        f();
    }

    void stop()
    {
        periodicTimer.stop();
        neutral();
        ROS_INFO("Periodic mode has finished.");
    }

    void start(const double delay)
    {
        startTimer = nh.createTimer(ros::Duration(delay), &PeriodicMode::start, this, true, true);
    }

    void start(const ros::TimerEvent& e)
    {
        start();
    }

    ~PeriodicMode() = default;
};


#endif  // PERIODICTARGETMODE_H