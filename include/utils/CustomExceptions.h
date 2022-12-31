#ifndef CUSTOMEXCEPTIONS_H
#define CUSTOMEXCEPTIONS_H


#include <utility>
#include <ros/ros.h>


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
    addition_error(const std::string& __arg) : auto_print_error(__arg) {}
};

class quaternion_error : public auto_print_error
{
public:
    quaternion_error() : auto_print_error("Quaternion is not unit quaternion!") {}
};


#endif  // CUSTOMEXCEPTIONS_H