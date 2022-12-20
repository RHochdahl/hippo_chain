#ifndef STATEPROVIDER_H
#define STATEPROVIDER_H


#include <Eigen/Dense>
#include <vector>

struct StateProvider
{
    const std::vector<double> pose;
    const std::vector<double> twist;


    StateProvider(const std::vector<double>& pose, const std::vector<double>& twist)
    : pose(pose)
    , twist(twist)
    {}

    StateProvider(const std::vector<double>& pose)
    : pose(pose)
    , twist(pose.size(), 0)
    {}

    StateProvider(const int size)
    : pose(size, 0)
    , twist(size, 0)
    {}

    StateProvider() {}


    template<typename T>
    T getPose() const
    {
        if constexpr (std::is_same<T, double>::value) {
            assert(pose.size() == 1);
            return pose.front();
        } else {
            assert(pose.size() == T::RowsAtCompileTime);
            return T(pose.data());
        }
    }

    template<typename T>
    T getTwist() const
    {
        if constexpr (std::is_same<T, double>::value) {
            assert(twist.size() == 1);
            return twist.front();
        } else {
            assert(twist.size() == T::RowsAtCompileTime);
            return T(twist.data());
        }
    }
};


#endif  // STATEPROVIDER_H