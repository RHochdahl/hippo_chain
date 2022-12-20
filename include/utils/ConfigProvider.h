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

    std::vector<std::string> getChainVehicleNames()
    {
        std::vector<std::string> names(0);
        
        std::vector<std::string> keys;
        if (nh->getParamNames(keys)) {
            for (auto it=keys.begin(); it!=keys.end(); it++) {
                std::size_t idx = it->find("/parent");
                if (idx == std::string::npos) continue;
                std::string base = it->substr(0, idx);
                names.push_back(base.substr(base.find_last_of("/")+1));
            }
        }
        
        return names;
    }

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