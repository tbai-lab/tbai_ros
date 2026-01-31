#include <ros/ros.h>
#include <tbai_ros_g1/G1ASAPMimicController.hpp>

namespace tbai {
namespace g1 {

RosG1ASAPMimicController::RosG1ASAPMimicController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                                                   const std::string &policyPath, float motionLength,
                                                   const std::string &controllerName)
    : G1ASAPMimicController(stateSubscriberPtr, policyPath, motionLength, controllerName) {}

void RosG1ASAPMimicController::preStep(scalar_t currentTime, scalar_t dt) {
    ros::spinOnce();
    G1ASAPMimicController::preStep(currentTime, dt);
}

bool RosG1ASAPMimicController::ok() const {
    return ros::ok();
}

}  // namespace g1
}  // namespace tbai
