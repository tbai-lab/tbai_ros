#include <ros/ros.h>
#include <tbai_ros_g1/G1PBHCController.hpp>

namespace tbai {
namespace g1 {

RosG1PBHCController::RosG1PBHCController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                                         const std::string &policyPath, const std::string &motionFilePath,
                                         float timeStart, float timeEnd, const std::string &controllerName)
    : G1PBHCController(stateSubscriberPtr, policyPath, motionFilePath, timeStart, timeEnd, controllerName) {}

void RosG1PBHCController::preStep(scalar_t currentTime, scalar_t dt) {
    ros::spinOnce();
    G1PBHCController::preStep(currentTime, dt);
}

bool RosG1PBHCController::ok() const {
    return ros::ok();
}

}  // namespace g1
}  // namespace tbai
