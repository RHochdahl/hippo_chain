#ifndef UNIVERSALJOINTMODEL_H
#define UNIVERSALJOINTMODEL_H


#include "JointModel.h"
#include <string>
#include <hippo_chain/include/common/sharedAlgorithms.hpp>
#include <hippo_chain/include/chain_estimator/kalman_filter/KalmanFilter.h>


#define USE_KALMAN_FILTER


/**
 * @brief 2D-universal joint located on the x-axis of the robot
 * 
 */
class UniversalJointModel : public JointModel<2>
{
private:
    std::array<Axis, 2> axes;   // axes of rotation
    double jointPosX;           // x-coordinates of joint in child frame

    Eigen::Vector2d phiMeas;
    Eigen::Vector2d omegaMeas;

#ifdef USE_KALMAN_FILTER
    KalmanFilter<2> filter;
#endif  // USE_KALMAN_FILTER


public:
    static constexpr const char* jointTypeName = "universal";


    UniversalJointModel(const std::shared_ptr<ConfigProvider>& configProvider)
#ifdef USE_KALMAN_FILTER
    : filter()
#endif  // USE_KALMAN_FILTER
    {
        std::vector<int> axesInt;
        if (!(configProvider->getValue("joint/x_pos", jointPosX) &&
              configProvider->getValue("joint/axes", axesInt)))
            ROS_FATAL("Could not retrieve joint parameters!");

        if (axesInt.size() != 2)
            ROS_FATAL("Array has unexpected size!");

        axes[0] = static_cast<Axis>(axesInt[0]);
        axes[1] = static_cast<Axis>(axesInt[1]);
    }


    void updateCoordinates(const geometry_msgs::PoseWithCovariance& childPose,
                           const geometry_msgs::PoseWithCovariance& parentPose)
    {
        const Eigen::Quaterniond qC(childPose.pose.orientation.w,
                                    childPose.pose.orientation.x,
                                    childPose.pose.orientation.y,
                                    childPose.pose.orientation.z);
        const Eigen::Quaterniond qP(parentPose.pose.orientation.w,
                                    parentPose.pose.orientation.x,
                                    parentPose.pose.orientation.y,
                                    parentPose.pose.orientation.z);
        const Eigen::Quaterniond diff = qP.conjugate()*qC;
        for (int i=0; i<2; i++) {
            switch (axes[i])
            {
            case Axis::x:
                phiMeas[i] = 2 * std::atan(diff.x()/diff.w());
                break;

            case Axis::y:
                phiMeas[i] = 2 * std::atan(diff.y()/diff.w());
                break;

            case Axis::z:
                phiMeas[i] = 2 * std::atan(diff.z()/diff.w());
                break;

            default:
                throw std::runtime_error("Unknown Axis");
            }
        }
    }

    void updateVelocities(const geometry_msgs::TwistWithCovariance& childTwist,
                          const geometry_msgs::TwistWithCovariance& parentTwist)
    {
        const geometry_msgs::Vector3 omegaC = childTwist.twist.angular;
        const geometry_msgs::Vector3 omegaP = parentTwist.twist.angular;
        const double cosAlpha = std::cos(phiMeas(0));
        const double sinAlpha = std::sin(phiMeas(0));
        const double cosBeta = std::cos(phiMeas(1));
        const double sinBeta = std::sin(phiMeas(1));
        switch (axes[0])
        {
        case Axis::x:
            switch (axes[1])
            {
            case Axis::x:
                throw std::runtime_error("Rotation Axes are identical");
                break;

            case Axis::y:
                omegaMeas[0] = (cosBeta*omegaC.x+sinBeta*omegaC.z) - omegaP.x;
                omegaMeas[1] = omegaC.y - (cosAlpha*omegaP.y+sinAlpha*omegaP.z);
                break;

            case Axis::z:
                omegaMeas[0] = (cosBeta*omegaC.x-sinBeta*omegaC.y) - omegaP.x;
                omegaMeas[1] = omegaC.z - (cosAlpha*omegaP.z-sinAlpha*omegaP.y);
                break;

            default:
                throw std::runtime_error("Unknown Axis");
            }        
            break;

        case Axis::y:
            switch (axes[1])
            {
            case Axis::x:
                omegaMeas[0] = (cosBeta*omegaC.y+sinBeta*omegaC.x) - omegaP.y;
                omegaMeas[1] = omegaC.x - (cosAlpha*omegaP.x-sinAlpha*omegaP.z);
                break;

            case Axis::y:
                throw std::runtime_error("Rotation Axes identical");
                break;

            case Axis::z:
                omegaMeas[0] = (cosBeta*omegaC.y+sinBeta*omegaC.x) - omegaP.y;
                omegaMeas[1] = omegaC.z - (cosAlpha*omegaP.z+sinAlpha*omegaP.x);
                break;

            default:
                throw std::runtime_error("Unknown Axis");
            }        
            break;

        case Axis::z:
            switch (axes[1])
            {
            case Axis::x:
                omegaMeas[0] = (cosBeta*omegaC.z+sinBeta*omegaC.y) - omegaP.z;
                omegaMeas[1] = omegaC.x - (cosAlpha*omegaP.x+sinAlpha*omegaP.y);
                break;

            case Axis::y:
                omegaMeas[0] = (cosBeta*omegaC.z-sinBeta*omegaC.x) - omegaP.z;
                omegaMeas[1] = omegaC.y - (cosAlpha*omegaP.y-sinAlpha*omegaP.x);
                break;

            case Axis::z:
                throw std::runtime_error("Rotation Axes are identical");
                break;

            default:
                throw std::runtime_error("Unknown Axis");
            }        
            break;

        default:
            throw std::runtime_error("Unknown Axis");
        }
    }

    void executeFilter()
    {
#ifdef USE_KALMAN_FILTER
        Eigen::Vector4d meas;
        meas.topRows<2>() = phiMeas;
        meas.bottomRows<2>() = omegaMeas;
        const Eigen::Vector4d belief = filter.update(meas);
        theta = belief.topRows<2>();
        zeta = belief.bottomRows<2>();
#else  // USE_KALMAN_FILTER
        theta = thetaMeas;
        zeta = zetaMeas;
#endif  // USE_KALMAN_FILTER
    }
};


#endif  // UNIVERSALJOINTMODEL_H