#ifndef DYNAMICRECONFIGUREMANAGER_H
#define DYNAMICRECONFIGUREMANAGER_H


#include <dynamic_reconfigure/server.h>
#include <type_traits>
#include <map>
#include <string>


template<typename ConfigType, bool CheckedUpdate = false>
class DynamicReconfigureManager
{
private:
    typedef boost::function<void(ConfigType &, uint32_t)> CallbackType;
    typedef boost::function<void(const ConfigType &, uint32_t)> ConstCallbackType;

    const std::string name;
    dynamic_reconfigure::Server<ConfigType> server;
    CallbackType f;
    std::map<const void* const, ConstCallbackType> callbacks;
    ConfigType lastConfig;


    void callback(ConfigType& config, uint32_t level)
    {
        callback(config, level, std::conditional_t<CheckedUpdate, std::true_type, std::false_type>{});
    }

    void callback(ConfigType& config, uint32_t level, std::true_type)
    {
        if (!config.update) return;
        callback(config, level, std::false_type{});
        config.update = false;
    }

    void callback(ConfigType& config, uint32_t level, std::false_type)
    {
        lastConfig = config;

        for (auto it=callbacks.begin(); it!=callbacks.end(); it++) {
            try {
                it->second(config, level);
            }
            catch (const std::exception& e)
            {
                std::cerr << e.what() << "\n";
            }
            catch (...)
            {}
        }
    }


public:
    DynamicReconfigureManager(const std::string& name)
    : name(name)
    , server(ros::NodeHandle(name))
    , f(boost::bind(&DynamicReconfigureManager::callback, this, _1, _2))
    , callbacks()
    {
        server.setCallback(f);
    }

    ~DynamicReconfigureManager()
    {}


    bool registerCallback(const ConstCallbackType& cb, const void* const obj)
    {
        return callbacks.insert({obj, boost::ref(cb)}).second;
    }

    void initCallback(const void* const obj)
    {
        try
        {
            callbacks.at(obj)(lastConfig, ~0);
        }
        catch(const std::out_of_range& e)
        {
            ROS_ERROR("Could not init dynamic reconfigure callback that has not been successfully registered yet!");
        }
    }

    void removeCallback(const void* const obj)
    {
        auto val = callbacks.erase(obj);
    }
};


#endif  // DYNAMICRECONFIGUREMANAGER_H