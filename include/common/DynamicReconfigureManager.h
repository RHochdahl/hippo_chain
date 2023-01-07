#ifndef DYNAMICRECONFIGUREMANAGER_H
#define DYNAMICRECONFIGUREMANAGER_H


#include <dynamic_reconfigure/server.h>
#include <type_traits>
#include <map>
#include <string>


template<typename ConfigType>
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
        if (!callbacks.insert({obj, boost::ref(cb)}).second) return false;
        cb(lastConfig, ~0);
        return true;
    }

    void removeCallback(const void* const obj)
    {
        auto val = callbacks.erase(obj);
    }

    void callback(ConfigType& config, uint32_t level)
    {
        if (!config.update) return;
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

        config.update = false;
    }
};


#endif  // DYNAMICRECONFIGUREMANAGER_H