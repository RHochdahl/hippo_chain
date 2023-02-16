#ifndef CHAINTARGETVISUALPOSE_H
#define CHAINTARGETVISUALPOSE_H


#include <ros/ros.h>
#include <geometry_msgs/Pose.h>


class VisualPose
{
private:
    ros::NodeHandle nh;
    ros::Publisher pub;

    geometry_msgs::Pose pose;

    std::shared_ptr<VisualPose> parent;

    double jointPos;


public:


    VisualPose(const std::string& ns, const double jointPos=std::nan("base"), std::shared_ptr<VisualPose> parent=nullptr)
    : nh(ns)
    , pub(nh.advertise<geometry_msgs::Pose>("target_pose", 1, true))
    , pose()
    , parent(parent)
    , jointPos(jointPos)
    {
        pose.orientation.w = 1.0;
    }

    ~VisualPose() = default;

    void publish()
    {
        pub.publish(pose);
    }

    const geometry_msgs::Pose& get() const
    {
        return pose;
    }

    void set(const geometry_msgs::Pose& _pose)
    {
        pose = _pose;
    }

    void set(const std::vector<double>& _pose)
    {
        if (_pose.size() == 7 && parent == nullptr) {
            assert(std::abs(_pose[4]) < 1e-12);
            assert(std::abs(_pose[5]) < 1e-12);
            pose.position.x = _pose[0];
            pose.position.y = _pose[1];
            pose.position.z = _pose[2];
            pose.orientation.w = _pose[3];
            pose.orientation.x = _pose[4];
            pose.orientation.y = _pose[5];
            pose.orientation.z = _pose[6];
        } else if (parent != nullptr) {
            auto parentPose = parent->get();
            assert(std::abs(parentPose.orientation.x) < 1e-12);
            assert(std::abs(parentPose.orientation.y) < 1e-12);
            assert(!std::isnan(jointPos));
            const double angle = _pose.front();
            const double halfAngle = 0.5*angle;
            const double halfSin = std::sin(halfAngle);
            const double halfCos = std::cos(halfAngle);
            pose.orientation.w = parentPose.orientation.w*halfCos - parentPose.orientation.z*halfSin;
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = parentPose.orientation.z*halfCos + parentPose.orientation.w*halfSin;
            const double delX = -jointPos * (1+std::cos(angle));
            const double delY = -jointPos * std::sin(angle);
            const double qwqw = parentPose.orientation.w * parentPose.orientation.w;
            const double qwqz2 = 2*parentPose.orientation.w * parentPose.orientation.z;
            const double qzqz = parentPose.orientation.z * parentPose.orientation.z;
            pose.position.x = parentPose.position.x + qwqw*delX - qwqz2*delY - qzqz*delX;
            pose.position.y = parentPose.position.y + qwqw*delY + qwqz2*delX - qzqz*delY;
            pose.position.z = parentPose.position.z;
        } else ROS_ERROR("Could not set target pose for '%s'!", nh.getNamespace().c_str());
    }
};


#endif  // CHAINTARGETVISUALPOSE_H