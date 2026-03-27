#pragma once

#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tbai_core/control/RobotInterface.hpp>

namespace tbai {

class RosChangeControllerSubscriber : public ChangeControllerSubscriber {
   public:
    RosChangeControllerSubscriber(ros::NodeHandle &nh, const std::string &topic);
    void triggerCallbacks() override;

   private:
    void controllerCallback(const std_msgs::String::ConstPtr &msg);

    ros::Subscriber controllerSubscriber_;
    std::string latestControllerType_;
};

}  // namespace tbai
