#include <ros/ros.h>
#include <tbai_ros_g1/G1ASAPController.hpp>

namespace tbai {
namespace g1 {

RosG1ASAPController::RosG1ASAPController(
    const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
    const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr, const std::string &policyPath,
    const std::string &controllerName)
    : G1ASAPController(stateSubscriberPtr, refVelGenPtr, policyPath, controllerName) {}

void RosG1ASAPController::preStep(scalar_t currentTime, scalar_t dt) {
    ros::spinOnce();
    G1ASAPController::preStep(currentTime, dt);
}

bool RosG1ASAPController::ok() const {
    return ros::ok();
}

}  // namespace g1
}  // namespace tbai
