#include <ros/ros.h>
#include <tbai_ros_g1_mujoco/G1BeyondMimicController.hpp>

namespace tbai {
namespace g1 {

RosG1BeyondMimicController::RosG1BeyondMimicController(const std::shared_ptr<tbai::RobotInterface> &robotInterfacePtr,
                                                       const std::string &policyPath, const std::string &controllerName,
                                                       bool useModelMetaConfig, float actionBeta)
    : G1BeyondMimicController(robotInterfacePtr, policyPath, controllerName, useModelMetaConfig, actionBeta) {}

void RosG1BeyondMimicController::preStep(scalar_t currentTime, scalar_t dt) {
    ros::spinOnce();
    G1BeyondMimicController::preStep(currentTime, dt);
}

bool RosG1BeyondMimicController::ok() const {
    return ros::ok();
}

}  // namespace g1
}  // namespace tbai
