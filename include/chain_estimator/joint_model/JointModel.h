#ifndef JOINTMODEL_H
#define JOINTMODEL_H


#include <hippo_chain/include/common/typedefs.h>
#include <hippo_chain/include/common/ConfigProvider.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>


template<std::size_t dof>
class JointModel
{
public:
    static constexpr int DOF = dof;
    typedef typename std::conditional<DOF == 1, double, Eigen::Matrix<double, DOF, 1>>::type JointVector;

    JointVector theta;                      // joint coordinates
    JointVector zeta;                       // joint velocities


    std::vector<double> coordsToVector() const
    {
        if constexpr (DOF == 1) {
            return std::vector<double>(1, theta);
        } else {
            return std::vector<double>(theta.data(), theta.data()+DOF);
        }
    }

    std::vector<double> velToVector() const
    {
        if constexpr (DOF == 1) {
            return std::vector<double>(1, zeta);
        } else {
            return std::vector<double>(zeta.data(), zeta.data()+DOF);
        }
    }


    virtual void executeFilter() = 0;

    virtual void updateCoordinates(const geometry_msgs::PoseWithCovariance& childPose,
                                   const geometry_msgs::PoseWithCovariance& parentPose) = 0;

    virtual void updateVelocities(const geometry_msgs::TwistWithCovariance& childTwist,
                                  const geometry_msgs::TwistWithCovariance& parentTwist) = 0;
};


#endif  // JOINTMODEL_H