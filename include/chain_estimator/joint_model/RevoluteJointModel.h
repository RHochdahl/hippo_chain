#ifndef REVOLUTEJOINTMODEL_H
#define REVOLUTEJOINTMODEL_H


#include "JointModel.h"
#include <string>
#include <hippo_chain/include/common/sharedAlgorithms.hpp>
#include <hippo_chain/include/chain_estimator/kalman_filter/KalmanFilter.h>


/**
 * @brief 1D-revolute joint located on the x-axis of the robot
 * 
 */
class RevoluteJointModel : public JointModel<1>
{
private:
    enum Axis
    {
        undefined = 0,
        x = 1,
        y = 2,
        z = 3
    } axis;                 // axis of rotation
    double jointPosX;       // x-coordinates of joint in child frame

    double phiMeas;
    double omegaMeas;

    KalmanFilter<1> filter;


public:
    static constexpr const char* jointTypeName = "revolute";


    RevoluteJointModel(const std::shared_ptr<ConfigProvider>& configProvider)
    : filter()
    {
        int axisInt;
        double poseLim, twistLim;
        if (!(configProvider->getValue("joint/x_pos", jointPosX) &&
              configProvider->getValue("joint/axis", axisInt) &&
              configProvider->getValue("joint/bounds/pose", poseLim) &&
              configProvider->getValue("joint/bounds/twist", twistLim)))
            ROS_FATAL("Could not retrieve joint parameters!");
        axis = static_cast<Axis>(axisInt);
    }


    void updateCoordinates(const geometry_msgs::PoseWithCovariance& childPose,
                           const geometry_msgs::PoseWithCovariance& parentPose)
    {
        const geometry_msgs::Quaternion qC = childPose.pose.orientation;
        const geometry_msgs::Quaternion qP = parentPose.pose.orientation;
        switch (axis)
        {
        case Axis::x:
            phiMeas = 2 * shared::sgn(qP.w*qC.x - qC.w*qP.x - qP.y*qC.z + qP.z*qC.y)
                      * std::acos(std::clamp(qC.w*qP.w + qC.x*qP.x + qC.y*qP.y + qC.z*qP.z, -1.0, 1.0));
            break;

        case Axis::y:
            phiMeas = 2 * shared::sgn(qP.w*qC.y - qC.w*qP.y - qP.z*qC.x + qP.x*qC.z)
                      * std::acos(std::clamp(qC.w*qP.w + qC.x*qP.x + qC.y*qP.y + qC.z*qP.z, -1.0, 1.0));
            break;

        case Axis::z:
            phiMeas = 2 * shared::sgn(qP.w*qC.z - qC.w*qP.z - qP.x*qC.y + qP.y*qC.x)
                      * std::acos(std::clamp(qC.w*qP.w + qC.x*qP.x + qC.y*qP.y + qC.z*qP.z, -1.0, 1.0));
            break;

        default:
            throw std::runtime_error("Unknown Axis");
        }
    }

    void updateVelocities(const geometry_msgs::TwistWithCovariance& childTwist,
                          const geometry_msgs::TwistWithCovariance& parentTwist)
    {
        const geometry_msgs::Vector3 omegaC = childTwist.twist.angular;
        const geometry_msgs::Vector3 omegaP = parentTwist.twist.angular;
        switch (axis)
        {
        case Axis::x:
            omegaMeas = omegaC.x-omegaP.x;
            break;

        case Axis::y:
            omegaMeas = omegaC.y-omegaP.y;
            break;

        case Axis::z:
            omegaMeas = omegaC.z-omegaP.z;
            break;

        default:
            throw std::runtime_error("Unknown Axis");
        }
    }

    void executeFilter()
    {
        const Eigen::Vector2d meas(phiMeas, omegaMeas);
        const Eigen::Vector2d belief = filter.update(meas);
        theta = belief(0);
        zeta = belief(1);
    }
};


#endif  // REVOLUTEJOINTMODEL_H