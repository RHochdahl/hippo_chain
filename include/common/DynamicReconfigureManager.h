#ifndef DYNAMICRECONFIGUREMANAGER_H
#define DYNAMICRECONFIGUREMANAGER_H


#include <dynamic_reconfigure/server.h>
#include <type_traits>
#include <map>
#include <string>

#include <hippo_chain/include/common/defines.h>

#include <fstream>



DEFINE_HAS_PARAM(update)
DEFINE_HAS_PARAM(save_config)
DEFINE_HAS_PARAM(clear)
DEFINE_HAS_PARAM(reset_to_default)


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
    ConfigType defaultConfig;
    uint32_t levelCombined;

    std::string configFilePath;


    void callback(ConfigType& config, uint32_t level)
    {
        levelCombined |= level;

        if constexpr (hippo::has_param_clear_v<ConfigType>()) if (config.clear) {
            config = lastConfig;
            levelCombined = 0;
            return;
        }

        if constexpr (hippo::has_param_reset_to_default_v<ConfigType>()) if (config.reset_to_default) {
            config = defaultConfig;
            callbackImpl(config, ~0);
            levelCombined = 0;
            return;
        }

        if constexpr (hippo::has_param_update_v<ConfigType>()) if (!config.update) return;
        else config.update = false;

        if constexpr (hippo::has_param_save_config_v<ConfigType>()) if (config.save_config) save(config);

        callbackImpl(config, levelCombined);
        levelCombined = 0;
    }

    void callbackImpl(ConfigType& config, uint32_t level)
    {
        lastConfig = config;

        for (auto it=callbacks.begin(); it!=callbacks.end(); it++) {
            try {
                it->second(config, level);
            }
            catch (const std::exception& e)
            {
                ROS_ERROR("Error in dynamic reconfigure callback: %s", e.what());
            }
            catch (...)
            {}
        }
    }

    void save(ConfigType& config)
    {
        ROS_INFO("Saving dynamic reconfigure parameters");

        config.save_config = false;

        if (configFilePath.empty()) {
            ROS_ERROR("Couldn't save config. No file path was specified.");
            return;
        }
        if(configFilePath.substr(configFilePath.find_last_of(".") + 1) != "csv") {
            ROS_ERROR("Couldn't save config. Specified file is not a csv file.");
            return;
        }

        dynamic_reconfigure::Config msg;
        config.__toMessage__(msg);

        std::ofstream configFile(configFilePath);

        for (auto it=msg.bools.begin(); it!=msg.bools.end(); it++) {
            configFile << "bool," << it->name << "," << std::to_string(it->value) << "\n";
        }
        for (auto it=msg.ints.begin(); it!=msg.ints.end(); it++) {
            configFile << "int," << it->name << "," << std::to_string(it->value) << "\n";
        }
        for (auto it=msg.doubles.begin(); it!=msg.doubles.end(); it++) {
            configFile << "double," << it->name << "," << std::to_string(it->value) << "\n";
        }
        for (auto it=msg.strs.begin(); it!=msg.strs.end(); it++) {
            configFile << "str," << it->name << "," << it->value << "\n";
        }
        for (auto it=msg.groups.begin(); it!=msg.groups.end(); it++) {
            configFile << "group," << it->name << "," << std::to_string(it->state) << "," << std::to_string(it->id) << "," << std::to_string(it->parent) << "\n";
        }

        configFile.close();
        ROS_INFO("Successfully saved dynamic reconfigure parameters to '%s'.", configFilePath.c_str());
    }

    bool load(ConfigType& config)
    {
        ROS_INFO("Loading dynamic reconfigure parameters");

        if (configFilePath.empty()) {
            ROS_ERROR("Couldn't save config. No file path was specified.");
            return false;
        }
        if (configFilePath.substr(configFilePath.find_last_of(".") + 1) != "csv") {
            ROS_ERROR("Couldn't save config. Specified file is not a csv file.");
            return false;
        }

        std::ifstream configFile(configFilePath);

        if (!configFile.is_open()) {
            ROS_ERROR("Could not open file '%s'", configFilePath.c_str());
            return false;
        }

        dynamic_reconfigure::Config msg;
        config.__toMessage__(msg);
        std::string line, entry;

        try
        {
            for (auto it=msg.bools.begin(); it!=msg.bools.end(); it++) {
                if (!std::getline(configFile, line))            throw std::runtime_error("More lines expected!");
                std::istringstream lineStream(line);
                if (!std::getline(lineStream, entry, ','))      throw std::runtime_error("Entry for type expected!");
                if (entry != "bool")                            throw std::runtime_error("Unexpected type '" + entry + "'! Expected 'bool'.");
                if (!std::getline(lineStream, entry, ','))      throw std::runtime_error("Entry for name expected!");
                if (entry != it->name)                          throw std::runtime_error("Unexpected name '" + entry + "' for bool param! '" + it->name + "' expected.");
                if (!std::getline(lineStream, entry, ','))      throw std::runtime_error("Entry for bool value expected!");
                it->value = std::stoi(entry);
                if (std::getline(lineStream, entry, ','))       throw std::runtime_error("Unexpected entry '" + entry + "'!");
            }
            for (auto it=msg.ints.begin(); it!=msg.ints.end(); it++) {
                if (!std::getline(configFile, line))            throw std::runtime_error("More lines expected!");
                std::istringstream lineStream(line);
                if (!std::getline(lineStream, entry, ','))      throw std::runtime_error("Entry for type expected!");
                if (entry != "int")                             throw std::runtime_error("Unexpected type '" + entry + "'! Expected 'int'.");
                if (!std::getline(lineStream, entry, ','))      throw std::runtime_error("Entry for name expected!");
                if (entry != it->name)                          throw std::runtime_error("Unexpected name '" + entry + "' for bool param! '" + it->name + "' expected.");
                if (!std::getline(lineStream, entry, ','))      throw std::runtime_error("Entry for int value expected!");
                it->value = std::stoi(entry);
                if (std::getline(lineStream, entry, ','))       throw std::runtime_error("Unexpected entry '" + entry + "'!");
            }
            for (auto it=msg.doubles.begin(); it!=msg.doubles.end(); it++) {
                if (!std::getline(configFile, line))            throw std::runtime_error("More lines expected!");
                std::istringstream lineStream(line);
                if (!std::getline(lineStream, entry, ','))      throw std::runtime_error("Entry for type expected!");
                if (entry != "double")                          throw std::runtime_error("Unexpected type '" + entry + "'! Expected 'double'.");
                if (!std::getline(lineStream, entry, ','))      throw std::runtime_error("Entry for name expected!");
                if (entry != it->name)                          throw std::runtime_error("Unexpected name '" + entry + "' for bool param! '" + it->name + "' expected.");
                if (!std::getline(lineStream, entry, ','))      throw std::runtime_error("Entry for double value expected!");
                it->value = std::stod(entry);
                if (std::getline(lineStream, entry, ','))       throw std::runtime_error("Unexpected entry '" + entry + "'!");
            }
            for (auto it=msg.strs.begin(); it!=msg.strs.end(); it++) {
                if (!std::getline(configFile, line))            throw std::runtime_error("More lines expected!");
                std::istringstream lineStream(line);
                if (!std::getline(lineStream, entry, ','))      throw std::runtime_error("Entry for type expected!");
                if (entry != "str")                             throw std::runtime_error("Unexpected type '" + entry + "'! Expected 'str'.");
                if (!std::getline(lineStream, entry, ','))      throw std::runtime_error("Entry for name expected!");
                if (entry != it->name)                          throw std::runtime_error("Unexpected name '" + entry + "' for bool param! '" + it->name + "' expected.");
                if (!std::getline(lineStream, it->value, ','))  throw std::runtime_error("Entry for string value expected!");;
                if (std::getline(lineStream, entry, ','))       throw std::runtime_error("Unexpected entry '" + entry + "'!");
            }
            for (auto it=msg.groups.begin(); it!=msg.groups.end(); it++) {
                if (!std::getline(configFile, line))            throw std::runtime_error("More lines expected!");
                std::istringstream lineStream(line);
                if (!std::getline(lineStream, entry, ','))      throw std::runtime_error("Entry for type expected!");
                if (entry != "group")                           throw std::runtime_error("Unexpected type '" + entry + "'! Expected 'group'.");
                if (!std::getline(lineStream, entry, ','))      throw std::runtime_error("Entry for name expected!");
                if (entry != it->name)                          throw std::runtime_error("Unexpected name '" + entry + "' for bool param! '" + it->name + "' expected.");
                if (!std::getline(lineStream, entry, ','))      throw std::runtime_error("Entry for state of group expected!");
                it->state = std::stoi(entry);
                if (!std::getline(lineStream, entry, ','))      throw std::runtime_error("Entry for group id expected!");
                it->id = std::stoi(entry);
                if (!std::getline(lineStream, entry, ','))      throw std::runtime_error("Entry for parent of group expected!");
                it->parent = std::stoi(entry);
                if (std::getline(lineStream, entry, ','))       throw std::runtime_error("Unexpected entry '" + entry + "'!");
            }

            if (std::getline(configFile, line))                 throw std::runtime_error("Unexpected line '" + line + "'!");
            configFile.close();
        }
        catch(const std::exception& e)
        {
            ROS_ERROR("Could not read dynamic reconfigure config from file: %s", e.what());
            configFile.close();
            return false;
        }

        config.__fromMessage__(msg);
        ROS_INFO("Successfully loaded dynamic reconfigure parameters from '%s'.", configFilePath.c_str());
        return true;
    }


public:
    DynamicReconfigureManager(const std::string& name, const ConfigType* const initConfig = nullptr, const std::string& configFilePath = "")
    : name(name)
    , server(ros::NodeHandle(name))
    , f(boost::bind(&DynamicReconfigureManager::callback, this, _1, _2))
    , callbacks()
    , lastConfig()
    , defaultConfig()
    , levelCombined(0)
    , configFilePath(configFilePath)
    {
        if (initConfig) {
            server.updateConfig(*initConfig);
            lastConfig = defaultConfig = *initConfig;
        } else if (!configFilePath.empty()) {
            ConfigType fileConfig;
            if (load(fileConfig)) {
                server.updateConfig(fileConfig);
                lastConfig = defaultConfig = fileConfig;
            }
        }
        server.setCallback(f);
    }

    DynamicReconfigureManager(const std::string& name, const std::string& configFilePath)
    : DynamicReconfigureManager(name, nullptr, configFilePath)
    {}

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
        callbacks.erase(obj);
    }
};


#endif  // DYNAMICRECONFIGUREMANAGER_H