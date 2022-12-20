#ifndef SHAREDALGORITHMS_H
#define SHAREDALGORITHMS_H


#include <ros/ros.h>
#include "typedefs.h"
#include "string.h"


namespace shared
{
    template <typename T>
    static inline int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }

    static inline Eigen::Vector3d cross3(const Eigen::Vector3d& first, const Eigen::Vector3d& second)
    {
        Eigen::Vector3d res;
        res(0) = first(1) * second(2) - first(2) * second(1);
        res(1) = first(2) * second(0) - first(0) * second(2);
        res(2) = first(0) * second(1) - first(1) * second(0);
        return res;
    }

    static inline Eigen::Vector6d cross6(const Eigen::Vector6d& first, const Eigen::Vector6d& second)
    {
        Eigen::Vector6d res;
        res.topRows(3) = cross3(first.bottomRows(3), second.topRows(3)) + cross3(first.topRows(3), second.bottomRows(3));
        res.bottomRows(3) = cross3(first.bottomRows(3), second.bottomRows(3));
        return res;
    }


    static inline int getID(const std::string& name)
    {
        try
        {
            return std::stoi(name.substr(name.find_last_not_of("0123456789") + 1));
        }
        catch(const std::invalid_argument& e)
        {
            ROS_ERROR("Could not read id from name '%s'!", name.c_str());
            return std::numeric_limits<int>::min();
        }
    }
}

#endif  // SHAREDALGORITHMS_H