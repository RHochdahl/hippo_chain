#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H


#include <ros/ros.h>
#include <memory>
#include "../../state/StateProvider.h"


template<typename T>
class Subscriber
{
protected:
    ros::Subscriber sub;
    bool updated;
    const std::shared_ptr<std::map<int, int>> idMap;
    std::vector<std::shared_ptr<StateProvider>> state;


    void setState(const int publicId, const std::shared_ptr<StateProvider> newState)
    {
        try
        {
            state[idMap->at(publicId)] = newState;
        }
        catch(const std::out_of_range& e)
        {
            ROS_WARN_THROTTLE(5.0, "vehicle with id %i not part of controlled chain!", publicId);
        }
    }


public:
    Subscriber(ros::NodeHandle& nh, const std::string& topic, const std::shared_ptr<std::map<int, int>> idMap)
    : sub(nh.subscribe(topic, 1, &Subscriber::callback, this))
    , updated(false)
    , idMap(idMap)
    , state(idMap->size())
    {}
    
    bool hasNewInputs() const
    {
        return updated;
    }

    const std::shared_ptr<StateProvider> getState(const int controllerId)
    {
        assert(id < state.size());
        updated = false;
        return state[controllerId];
    }

    const void reset()
    {
        state.resize(idMap->size());
        updated = false;
    }

    virtual void callback(const T& msg) = 0;
};


#endif  // SUBSCRIBER_H