#include "tbai_ros_wtw/WtwController.hpp"

#include <tbai_core/config/Config.hpp>

namespace tbai {
namespace wtw {

RosWtwController::RosWtwController(const std::string &urdfString,
                                   const std::shared_ptr<tbai::RobotInterface> &robotInterfacePtr,
                                   const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr)
    : tbai::WtwController(urdfString, robotInterfacePtr, refVelGenPtr) {
    timeSinceLastVisualizationUpdate_ = 1000.0;
}

void RosWtwController::postStep(scalar_t currentTime, scalar_t dt) {
    if (timeSinceLastVisualizationUpdate_ >= 1.0 / 30.0) {
        ::tbai::wtw::State &state = wtwState_;
        stateVisualizer_.visualize(state);
        contactVisualizer_.visualize(state_.x, state_.contactFlags);
        timeSinceLastVisualizationUpdate_ = 0.0;
    } else {
        timeSinceLastVisualizationUpdate_ += dt;
    }
}

void RosWtwController::changeController(const std::string &controllerType, scalar_t currentTime) {
    preStep(currentTime, 0.0);
}

void RosWtwController::stopController() {}

bool RosWtwController::ok() const {
    return ros::ok();
}

void RosWtwController::preStep(scalar_t currentTime, scalar_t dt) {
    ros::spinOnce();
    state_ = robotInterfacePtr_->getLatestState();
}

}  // namespace wtw
}  // namespace tbai
