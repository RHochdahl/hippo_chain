#ifndef STATEPROVIDER_H
#define STATEPROVIDER_H


#include <Eigen/Dense>
#include <boost/array.hpp>

struct StateProvider
{
    const boost::array<double, 7> pose;
    const boost::array<double, 6> twist;


    StateProvider(const boost::array<double, 7>& pose, const boost::array<double, 6>& twist)
    : pose(pose)
    , twist(twist)
    {}

    StateProvider(const boost::array<double, 7>& pose)
    : pose(pose)
    , twist()
    {}

    StateProvider()
    : pose()
    , twist()
    {}


    template<std::size_t ARRAY_SIZE, std::size_t EXPECTED_SIZE>
    static inline void assertSize(const boost::array<double, ARRAY_SIZE>& arr)
    {
        static_assert(EXPECTED_SIZE <= ARRAY_SIZE);
#ifndef NDEBUG
        for (std::size_t idx=EXPECTED_SIZE; idx < ARRAY_SIZE; idx++) {
            assert(std::abs(arr[idx]) == 0.0);
        }
#endif  // NDEBUG
    }

    template<typename T>
    T getPose() const
    {
        if constexpr (std::is_same<T, double>::value) {
            assertSize<7,1>(pose);
            return pose.front();
        } else {
            assertSize<7,T::RowsAtCompileTime>(pose);
            return T(pose.data());
        }
    }

    template<typename T>
    T getTwist() const
    {
        if constexpr (std::is_same<T, double>::value) {
            assertSize<6,1>(twist);
            return twist.front();
        } else {
            assertSize<6,T::RowsAtCompileTime>(twist);
            return T(twist.data());
        }
    }
};


#endif  // STATEPROVIDER_H