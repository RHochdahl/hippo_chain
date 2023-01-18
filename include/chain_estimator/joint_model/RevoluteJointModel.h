#ifndef REVOLUTEJOINTMODEL_H
#define REVOLUTEJOINTMODEL_H


#include "JointModel.h"
#include <string>
#include <hippo_chain/include/common/sharedAlgorithms.hpp>


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

public:
    static constexpr const char* jointTypeName = "revolute";


    RevoluteJointModel(const std::shared_ptr<ConfigProvider>& configProvider)
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
            theta = 2 * shared::sgn(qP.w*qC.x - qC.w*qP.x - qP.y*qC.z + qP.z*qC.y)
                      * std::acos(std::clamp(qC.w*qP.w + qC.x*qP.x + qC.y*qP.y + qC.z*qP.z, -1.0, 1.0));
            break;

        case Axis::y:
            theta = 2 * shared::sgn(qP.w*qC.y - qC.w*qP.y - qP.z*qC.x + qP.x*qC.z)
                      * std::acos(std::clamp(qC.w*qP.w + qC.x*qP.x + qC.y*qP.y + qC.z*qP.z, -1.0, 1.0));
            break;

        case Axis::z:
            theta = 2 * shared::sgn(qP.w*qC.z - qC.w*qP.z - qP.x*qC.y + qP.y*qC.x)
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
            zeta = omegaC.x-omegaP.x;
            break;

        case Axis::y:
            zeta = omegaC.y-omegaP.y;
            break;

        case Axis::z:
            zeta = omegaC.z-omegaP.z;
            break;

        default:
            throw std::runtime_error("Unknown Axis");
        }
    }
};


#endif  // REVOLUTEJOINTMODEL_H