#ifndef JOINTMODEL_H
#define JOINTMODEL_H


#include <hippo_chain/include/common/typedefs.h>
#include <hippo_chain/include/common/ConfigProvider.h>
#include <hippo_chain/include/common/sharedAlgorithms.hpp>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>


template<std::size_t dof>
class JointModel
{
protected:
    enum Axis
    {
        undefined = 0,
        x = 1,
        y = 2,
        z = 3
    };


public:
    static constexpr int DOF = dof;
    typedef typename std::conditional<DOF == 1, double, Eigen::Matrix<double, DOF, 1>>::type JointVector;

    JointVector theta;                      // joint coordinates
    JointVector zeta;                       // joint velocities


    boost::array<double,7> coordsToVector() const
    {
        if constexpr (DOF == 1) {
            return boost::array<double,7>({theta});
        } else {
            return shared::toArray<7>(theta);
        }
    }

    boost::array<double,6> velToVector() const
    {
        if constexpr (DOF == 1) {
            return boost::array<double,6>({zeta});
        } else {
            return shared::toArray<6>(zeta);
        }
    }


    virtual void executeFilter() = 0;

    virtual void updateCoordinates(const geometry_msgs::PoseWithCovariance& childPose,
                                   const geometry_msgs::PoseWithCovariance& parentPose) = 0;

    virtual void updateVelocities(const geometry_msgs::TwistWithCovariance& childTwist,
                                  const geometry_msgs::TwistWithCovariance& parentTwist) = 0;
};


#endif  // JOINTMODEL_H