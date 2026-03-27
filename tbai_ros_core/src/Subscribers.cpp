#include "tbai_ros_core/Subscribers.hpp"

#include <std_msgs/String.h>

namespace tbai {

RosChangeControllerSubscriber::RosChangeControllerSubscriber(ros::NodeHandle &nh, const std::string &topic) {
    controllerSubscriber_ = nh.subscribe(topic, 1, &RosChangeControllerSubscriber::controllerCallback, this);
}

void RosChangeControllerSubscriber::triggerCallbacks() {
    if (latestControllerType_.empty()) {
        return;
    }
    callbackFunction_(latestControllerType_);
    latestControllerType_.clear();
}

void RosChangeControllerSubscriber::controllerCallback(const std_msgs::String::ConstPtr &msg) {
    latestControllerType_ = msg->data;
}

}  // namespace tbai
