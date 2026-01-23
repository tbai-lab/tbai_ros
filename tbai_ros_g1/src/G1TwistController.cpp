#include <ros/ros.h>
#include <tbai_ros_g1/G1TwistController.hpp>

namespace tbai {
namespace g1 {

RosG1TwistController::RosG1TwistController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                                           const std::string &policyPath, const std::string &motionFilePath,
                                           float timeStart, float timeEnd, const std::string &controllerName)
    : G1TwistController(stateSubscriberPtr, policyPath, motionFilePath, timeStart, timeEnd, controllerName) {
}

void RosG1TwistController::preStep(scalar_t currentTime, scalar_t dt) {
    ros::spinOnce();
    G1TwistController::preStep(currentTime, dt);
}

bool RosG1TwistController::ok() const {
    return ros::ok();
}

}  // namespace g1
}  // namespace tbai
