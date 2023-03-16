#ifndef CHAINNODETEMPLATE_H
#define CHAINNODETEMPLATE_H


#include <utility>
#include <memory>
#include <ros/ros.h>
#include <hippo_chain/AddVehicles.h>
#include <std_srvs/Empty.h>

#include <hippo_chain/include/common/ConfigProvider.h>
#include <hippo_chain/include/common/sharedAlgorithms.hpp>


class ChainNodeTemplate
{
protected:
    std::shared_ptr<ros::NodeHandle> nh;
    ConfigProvider configProvider;                  // could be private
    ros::ServiceServer addVehiclesSrv;              // could be private
    ros::Rate rate;

    int numVehicles;
    std::shared_ptr<std::map<int, int>> idMap;
    std::shared_ptr<std::vector<int>> idList;
    std::vector<int> dofList;

    struct VehicleParam
    {
        std::string name;
        int publicId;
        int parentId;
        std::string jointType;
        int dof;
        double jointPos;
    };


    void readVehicleParams(const std::string& name, VehicleParam& param) const
    {
        param.name = name;
        param.publicId = shared::getID(param.name);
        if (param.publicId < 0)                                                         ROS_FATAL("ID could not be read from '%s'!", param.name.c_str());
        std::string parentName;
        if (!configProvider.getValue(param.name + "/parent", parentName))               ROS_FATAL("Vehicle '%s' does not name a parent!", param.name.c_str());
        if (parentName == "base") {
            param.parentId = -1;
            param.dof = 6;
        } else {
            param.parentId = shared::getID(parentName);
            if (param.parentId < 0)                                                     ROS_FATAL("ID could not be read from '%s'!", parentName.c_str());
            if (!configProvider.getValue(param.name + "/joint/type", param.jointType))  ROS_FATAL("Vehicle '%s' does not specify 'joint/type'!", param.name.c_str());
            if (!configProvider.getValue(param.name + "/joint/dof", param.dof))         ROS_FATAL("Vehicle '%s' does not specify 'joint/dof'!", param.name.c_str());
            if (!configProvider.getValue(param.name + "/joint/x_pos", param.jointPos))  ROS_FATAL("Vehicle '%s' does not specify 'joint/x_pos'!", param.name.c_str());
        }
    }

    bool addVehicles(const std::vector<std::string>& vehicles)
    {
        std::vector<VehicleParam> waitingList(0);

        for (auto it=vehicles.begin(); it!=vehicles.end(); it++) {
            VehicleParam param;
            readVehicleParams(*it, param);
            waitingList.push_back(param);
        }

        {
            auto it=waitingList.begin();
            while(it!=waitingList.end()) {
                if (it->parentId == -1) {
                    try
                    {
                        addBaseVehicle(*it);
                    }
                    catch(const auto_print_error& e) {}
                    catch(const std::exception& e) {ROS_FATAL("%s", e.what());}
                    it = waitingList.erase(it);
                } else it++;
            }
        }
        if (!numVehicles) ROS_FATAL("No base vehicle could be constructed!");
        {
            auto it=waitingList.begin();
            while(it!=waitingList.end()) {
                if (idMap->count(it->parentId)) {
                    try
                    {
                        addChildVehicle(*it);
                    }
                    catch(const auto_print_error& e) {}
                    catch(const std::exception& e) {ROS_FATAL("%s", e.what());}
                    waitingList.erase(it);
                    it=waitingList.begin();
                } else it++;
            }
        }

        if (waitingList.size()) {
            ROS_ERROR("Some vehicles could not be added to chain!");
            return false;
        }
        return true;
    }

    bool addVehiclesCallback(hippo_chain::AddVehicles::Request& request, hippo_chain::AddVehicles::Response& response)
    {
        return addVehicles(request.vehicle_names);
    }

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


    virtual void addChildVehicle(const VehicleParam& param) = 0;
    virtual void addBaseVehicle(const VehicleParam& param) = 0;


public:
    ChainNodeTemplate(const std::string& name, std::vector<std::string>& vehicles, const double rate)
    : nh(new ros::NodeHandle())
    , configProvider(nh)
    , addVehiclesSrv(nh->advertiseService(name + "/addVehicles", &ChainNodeTemplate::addVehiclesCallback, this))
    , rate((std::isnan(rate) ? DEFAULT_RATE : rate))
    , numVehicles(0)
    , idMap(new std::map<int, int>())
    , idList(new std::vector<int>())
    , dofList()
    {
        if (!vehicles.size()) {
            vehicles = getChainVehicleNames();
            if (!vehicles.size()) ROS_ERROR("No vehicle names were given!");
        }
    }

    ~ChainNodeTemplate()
    {
        nh->shutdown();
    }

    virtual void spin() = 0;
};


#endif  // CHAINNODETEMPLATE_H