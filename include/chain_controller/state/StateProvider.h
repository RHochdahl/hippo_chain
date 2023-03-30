#ifndef STATEPROVIDER_H
#define STATEPROVIDER_H


#include <Eigen/Dense>
#include <boost/array.hpp>

struct StateProvider
{
    const boost::array<double, 7> pose;
    const boost::array<double, 6> twist;
    const boost::array<double, 6> accel;


    StateProvider(const boost::array<double, 7>& pose, const boost::array<double, 6>& twist, const boost::array<double, 6>& accel)
    : pose(pose)
    , twist(twist)
    , accel(accel)
    {}

    StateProvider(const boost::array<double, 7>& pose, const boost::array<double, 6>& twist)
    : pose(pose)
    , twist(twist)
    , accel()
    {}

    StateProvider(const boost::array<double, 7>& pose)
    : pose(pose)
    , twist()
    , accel()
    {}

    StateProvider()
    : pose()
    , twist()
    , accel()
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


#define DEFINE_GET_STATE(name, dim) \
    template<typename T>\
    T get_##name() const\
    {\
        if constexpr (std::is_same<T, double>::value) {\
            assertSize<dim,1>(name);\
            return name.front();\
        } else {\
            assertSize<dim,T::RowsAtCompileTime>(name);\
            return T(name.data());\
        }\
    }

DEFINE_GET_STATE(pose, 7)
DEFINE_GET_STATE(twist, 6)
DEFINE_GET_STATE(accel, 6)

};


#endif  // STATEPROVIDER_H