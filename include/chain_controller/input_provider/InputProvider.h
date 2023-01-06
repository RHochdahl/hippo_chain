#ifndef INPUTPROVIDER_H
#define INPUTPROVIDER_H


#include <utility>
#include <ros/ros.h>
#include <hippo_chain/include/chain_controller/state/StateProvider.h>
#include "subscriber/StateSubscriber.h"
#include "subscriber/TargetSubscriber.h"


class InputProvider
{
protected:
    const std::shared_ptr<std::map<int, int>> idMap;
    ros::NodeHandle nh;
    StateSubscriber stateSub;
    TargetSubscriber targetSub;
public:
    InputProvider(const std::shared_ptr<std::map<int, int>> idMap)
    : idMap(idMap)
    , nh("chain")
    , stateSub(nh, idMap)
    , targetSub(nh, idMap)
    {}

    bool hasNewInputs() const
    {
        return stateSub.hasNewInputs() || targetSub.hasNewInputs();
    }

    bool isUnsafe() const
    {
        return stateSub.isUnsafe();
    }

    const std::shared_ptr<StateProvider> getState(const int controllerId)
    {
        return stateSub.getState(controllerId);
    }

    const std::shared_ptr<StateProvider> getTarget(const int controllerId)
    {
        return targetSub.getState(controllerId);
    }

    const void reset()
    {
        stateSub.reset();
        targetSub.reset();
    }
};


#endif  // INPUTPROVIDER_H