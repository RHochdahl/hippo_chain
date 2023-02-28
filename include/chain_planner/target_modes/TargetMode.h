#ifndef TARGETMODE_H
#define TARGETMODE_H


#include <ros/ros.h>


class TargetMode
{
protected:
    ros::NodeHandle nh;
    ros::Timer startTimer;
    ros::Timer stopTimer;
    bool finished;

    void stop(const ros::TimerEvent& e)
    {
        stop();
    }


public:
    TargetMode(const double _duration)
    : nh()
    , stopTimer(nh.createTimer(ros::Duration(_duration), &TargetMode::stop, this, false, false))
    , finished(false)
    {}

    TargetMode()
    : nh()
    , stopTimer()
    , finished(false)
    {}

    ~TargetMode() = default;


    virtual bool set()
    {
        return finished;
    }

    virtual void start()
    {
        if (stopTimer.isValid()) stopTimer.start();
    }

    void stop()
    {
        finished = true;
        ROS_INFO("Mode has finished.");
    }

    void start(const double delay)
    {
        startTimer = nh.createTimer(ros::Duration(delay), &TargetMode::start, this, true, true);
    }

    void start(const ros::TimerEvent& e)
    {
        start();
    }
};

#endif  // TARGETMODE_H