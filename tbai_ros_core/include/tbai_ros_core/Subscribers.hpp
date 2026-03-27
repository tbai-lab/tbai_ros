#pragma once

#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tbai_core/control/RobotInterface.hpp>

namespace tbai {

class RosChangeControllerSubscriber : public ChangeControllerSubscriber {
   public:
    RosChangeControllerSubscriber(ros::NodeHandle &nh, const std::string &topic) {
        controllerSubscriber_ = nh.subscribe(topic, 1, &RosChangeControllerSubscriber::controllerCallback, this);
    }

    void triggerCallbacks() override {
        if (latestControllerType_.empty()) {
            return;
        }
        callbackFunction_(latestControllerType_);
        latestControllerType_.clear();
    }

   private:
    void controllerCallback(const std_msgs::String::ConstPtr &msg) { latestControllerType_ = msg->data; }

    ros::Subscriber controllerSubscriber_;
    std::string latestControllerType_;
};

}  // namespace tbai
