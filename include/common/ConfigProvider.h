#ifndef CONFIGPROVIDER_H
#define CONFIGPROVIDER_H


#include <memory>
#include <string>
#include <array>
#include <ros/ros.h>


class ConfigProvider
{
private:
    std::shared_ptr<ros::NodeHandle> nh;

public:
    ConfigProvider(std::shared_ptr<ros::NodeHandle> nh)
    : nh(nh)
    {}

    ~ConfigProvider()
    {}

    template<typename T>
    bool getValue(const std::string& name, T& variable) const
    {
        if (!nh->getParamCached(name, variable)) {
            ROS_ERROR("parameter %s' not found!", name.c_str());
            return false;
        }
        return true;
    }

    template<typename T>
    T getValueWithDefault(const std::string& name, const T& defaultValue) const
    {
        T variable;
        if (getValue(name, variable)) return variable;
        return defaultValue;
    }

    template<typename T>
    bool getValuePositive(const std::string& name, T& variable) const
    {
        if (getValue(name, variable)) {
            if (variable >= 0) return true;
            ROS_ERROR("parameter '%s' is negative!", name.c_str());
            return false;
        }
        return false;
    }

    template<typename T>
    T getValuePositiveWithDefault(const std::string& name, const T& defaultValue) const
    {
        T variable;
        if (getValuePositive(name, variable)) return variable;
        return defaultValue;
    }

    template<typename T>
    bool getAbsValue(const std::string& name, T& variable) const
    {
        if (getValue(name, variable)) {
            if (variable < 0) {
                variable *= -1;
                ROS_ERROR("parameter '%s' is negative! Returning abs value.", name.c_str());
            }
            return true;
        }
        return false;
    }

    template<typename T>
    T getAbsValueWithDefault(const std::string& name, const T& defaultValue) const
    {
        T variable;
        if (getAbsValue(name, variable)) return variable;
        return defaultValue;
    }

};


#endif  // CONFIGPROVIDER_H