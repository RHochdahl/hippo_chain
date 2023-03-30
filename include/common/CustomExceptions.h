#ifndef CUSTOMEXCEPTIONS_H
#define CUSTOMEXCEPTIONS_H


#include <utility>
#include <ros/ros.h>
#include <Eigen/Dense>


class auto_print_error : public std::runtime_error
{
public:
    auto_print_error(const std::string& __arg) : std::runtime_error(__arg)
    {
        ROS_ERROR("%s", __arg.c_str());
    }
};


class addition_error : public auto_print_error
{
public:
    addition_error(const std::string& __arg) : auto_print_error("Couldn't add vehicle! " + __arg) {}
};

class quaternion_error : public auto_print_error
{
public:
    quaternion_error() : auto_print_error("Quaternion is not unit quaternion!") {}
    quaternion_error(const Eigen::Vector4d& quat) : auto_print_error("Quaternion is not unit quaternion (w: " + std::to_string(quat(0)) +
                                                                                                     ",\tx: " + std::to_string(quat(1)) +
                                                                                                     ",\ty: " + std::to_string(quat(2)) +
                                                                                                     ",\tz: " + std::to_string(quat(3)) +
                                                                                                     ",\tnorm: " + std::to_string(quat.norm()) + ")!") {}
    quaternion_error(const Eigen::Quaterniond& quat) : auto_print_error("Quaternion is not unit quaternion (w: " + std::to_string(quat.w()) +
                                                                                                        ",\tx: " + std::to_string(quat.x()) +
                                                                                                        ",\ty: " + std::to_string(quat.y()) +
                                                                                                        ",\tz: " + std::to_string(quat.w()) +
                                                                                                        ",\tnorm: " + std::to_string(quat.norm()) + ")!") {}
};

class timeout_error : public auto_print_error
{
public:
    timeout_error(const std::string& __arg) : auto_print_error(__arg + " timed out!") {}
};


#endif  // CUSTOMEXCEPTIONS_H