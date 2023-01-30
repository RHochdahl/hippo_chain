#ifndef STATESUBSCRIBER_H
#define STATESUBSCRIBER_H


#include "Subscriber.h"
#include <hippo_chain/ChainState.h>


class StateSubscriber : public Subscriber<hippo_chain::ChainState>
{
protected:
    ros::Timer timer;
    bool timedOut;
    bool outOfBounds;
public:
    StateSubscriber(ros::NodeHandle& nh, const std::shared_ptr<std::map<int, int>> idMap)
    : Subscriber(nh, "state", idMap)
    , timer(nh.createTimer(ros::Rate(5.0), &StateSubscriber::bark, this, true, false))
    , timedOut(true)
    {}

    bool isUnsafe() const
    {
        return timedOut;
    }

    void callback(const hippo_chain::ChainState& msg)
    {
        timer.stop();
        for (auto vehicleState : msg.data) {
            setState(vehicleState.vehicle_id,
                     std::make_shared<StateProvider>(vehicleState.pose, vehicleState.twist));
        }
        updated = true;
        timedOut = false;
        timer.start();
    }

    void bark(const ros::TimerEvent& e)
    {
        ROS_WARN_THROTTLE(5.0, "State too old, setting target velocity to zero.");
        timedOut = true;
    }
};



#endif  // STATESUBSCRIBER_H