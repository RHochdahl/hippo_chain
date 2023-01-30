#ifndef VEHICLEWATCHER_H
#define VEHICLEWATCHER_H


#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <hippo_chain/include/chain_watcher/Boundaries.h>
#include <hippo_chain/include/common/CustomExceptions.h>


class VehicleWatcher
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Timer timer;

    const std::shared_ptr<Boundaries const> bounds;


    void callback(const nav_msgs::Odometry& msg)
    {
        timer.stop();

        geometry_msgs::Point pos = msg.pose.pose.position;
        if (pos.x < bounds->x.lower ||
            pos.x > bounds->x.upper ||
            pos.y < bounds->y.lower ||
            pos.y > bounds->y.upper ||
            pos.z < bounds->z.lower ||
            pos.z > bounds->z.upper) {
            
            throw auto_print_error("Vehicle '" + nh.getNamespace() + "' out of bounds!");
        }

        timer.start();
    }

    void bark(const ros::TimerEvent& e)
    {
        throw auto_print_error("No odom received from vehicle '" + nh.getNamespace() + "'!");
    }


public:
    VehicleWatcher(const std::string& name, const std::shared_ptr<Boundaries const> bounds_ptr)
    : nh(name)
    , sub(nh.subscribe("ground_truth/odom", 1, &VehicleWatcher::callback, this))
    , timer(nh.createTimer(ros::Rate(5.0), &VehicleWatcher::bark, this, true, false))
    , bounds(bounds_ptr)
    {}

    ~VehicleWatcher() {}


    std::string getName() const
    {
        return nh.getNamespace();
    }
};


#endif  // VEHICLEWATCHER_H