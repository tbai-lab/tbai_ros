#include <ros/ros.h>
#include <tbai_ros_g1/G1Twist2Controller.hpp>

namespace tbai {
namespace g1 {

RosG1Twist2Controller::RosG1Twist2Controller(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                                           const std::string &policyPath, const std::string &motionFilePath,
                                           float timeStart, float timeEnd, const std::string &controllerName)
    : G1Twist2Controller(stateSubscriberPtr, policyPath, motionFilePath, timeStart, timeEnd, controllerName) {
}

void RosG1Twist2Controller::preStep(scalar_t currentTime, scalar_t dt) {
    ros::spinOnce();
    G1Twist2Controller::preStep(currentTime, dt);
}

bool RosG1Twist2Controller::ok() const {
    return ros::ok();
}

}  // namespace g1
}  // namespace tbai
