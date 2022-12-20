#ifndef TARGETSUBSCRIBER_H
#define TARGETSUBSCRIBER_H


#include "Subscriber.h"
#include <hippo_chain/ChainState.h>


class TargetSubscriber : public Subscriber<hippo_chain::ChainState>
{
protected:
    ros::Timer timer;
public:
    TargetSubscriber(ros::NodeHandle& nh, const std::shared_ptr<std::map<int, int>> idMap)
    : Subscriber(nh, "target", idMap)
    , timer(nh.createTimer(ros::Duration(1), &TargetSubscriber::bark, this, true, false))
    {}

    void callback(const hippo_chain::ChainState& msg)
    {
        timer.stop();
        for (auto vehicleState : msg.data) {
            setState(vehicleState.vehicle_id,
                     std::make_shared<StateProvider>(vehicleState.pose, vehicleState.twist));
        }
        updated = true;
        timer.start();
    }

    void bark(const ros::TimerEvent& e)
    {
        for (auto it=state.begin(); it!=state.end(); it++) {
            it->reset(new StateProvider((*it)->pose));
        }
        ROS_WARN_THROTTLE(5.0, "Target too old, setting target velocity to zero.");
        updated = true;
    }
};



#endif  // TARGETSUBSCRIBER_H