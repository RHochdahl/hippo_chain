#ifndef CHAINTARGETVISUALPOSE_H
#define CHAINTARGETVISUALPOSE_H


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


class VisualPose
{
private:
    ros::NodeHandle nh;
    ros::Publisher pub;

    geometry_msgs::PoseStamped pose;
    std::vector<double> poseVec;

    std::shared_ptr<VisualPose> parent;

    double jointPos;


public:


    VisualPose(const std::string& ns, const double jointPos=std::nan("base"), std::shared_ptr<VisualPose> parent=nullptr)
    : nh(ns)
    , pub(nh.advertise<geometry_msgs::PoseStamped>("target_pose", 1, true))
    , pose()
    , poseVec()
    , parent(parent)
    , jointPos(jointPos)
    {
        pose.header.frame_id = "map";
        pose.pose.orientation.w = 1.0;

        if (parent == nullptr) poseVec = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
        else poseVec = {0.0};
    }

    ~VisualPose() = default;

    void publish()
    {
        pose.header.stamp = ros::Time::now();
        pub.publish(pose);
    }

    const geometry_msgs::Pose& get() const
    {
        return pose.pose;
    }

    void set(const geometry_msgs::Pose& _pose)
    {
        pose.pose = _pose;
    }

    void set(const std::vector<double>& _pose)
    {
        if ((_pose.size() == 7 && parent == nullptr) || (parent != nullptr)) {
            poseVec = _pose;
            refresh();
        } else ROS_ERROR("Could not set target pose for '%s'!", nh.getNamespace().c_str());
    }

    void refresh()
    {
        if (parent == nullptr) {
            assert(std::abs(poseVec[4]) < 1e-12);
            assert(std::abs(poseVec[5]) < 1e-12);
            pose.pose.position.x = poseVec[0];
            pose.pose.position.y = poseVec[1];
            pose.pose.position.z = poseVec[2];
            pose.pose.orientation.w = poseVec[3];
            pose.pose.orientation.x = poseVec[4];
            pose.pose.orientation.y = poseVec[5];
            pose.pose.orientation.z = poseVec[6];
        } else {
            auto parentPose = parent->get();
            assert(std::abs(parentPose.orientation.x) < 1e-12);
            assert(std::abs(parentPose.orientation.y) < 1e-12);
            assert(!std::isnan(jointPos));
            const double angle = poseVec.back();
            const double halfAngle = 0.5*angle;
            const double halfSin = std::sin(halfAngle);
            const double halfCos = std::cos(halfAngle);
            pose.pose.orientation.w = parentPose.orientation.w*halfCos - parentPose.orientation.z*halfSin;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = parentPose.orientation.z*halfCos + parentPose.orientation.w*halfSin;
            const double delX = -jointPos * (1+std::cos(angle));
            const double delY = -jointPos * std::sin(angle);
            const double qwqw = parentPose.orientation.w * parentPose.orientation.w;
            const double qwqz2 = 2*parentPose.orientation.w * parentPose.orientation.z;
            const double qzqz = parentPose.orientation.z * parentPose.orientation.z;
            pose.pose.position.x = parentPose.position.x + qwqw*delX - qwqz2*delY - qzqz*delX;
            pose.pose.position.y = parentPose.position.y + qwqw*delY + qwqz2*delX - qzqz*delY;
            pose.pose.position.z = parentPose.position.z;
        }
    }
};


#endif  // CHAINTARGETVISUALPOSE_H