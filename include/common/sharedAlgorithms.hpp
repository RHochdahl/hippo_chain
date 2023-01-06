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

    static inline bool isUnitQuaternion(const Eigen::Vector4d& quat)
    {
        return (std::abs(quat.norm() - 1.0) < 1e-12);
    }

    static inline Eigen::Vector6d cross6(const Eigen::Vector6d& first, const Eigen::Vector6d& second)
    {
        Eigen::Vector6d res;
        res.topRows<3>() = first.bottomRows<3>().cross(second.topRows<3>()) + first.topRows<3>().cross(second.bottomRows<3>());
        res.bottomRows<3>() = first.bottomRows<3>().cross(second.bottomRows<3>());
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