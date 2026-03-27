#include "tbai_ros_bob/BobController.hpp"

#include <tbai_core/config/Config.hpp>

namespace tbai {
namespace rl {

RosBobController::RosBobController(const std::string &urdfString,
                                   const std::shared_ptr<tbai::RobotInterface> &robotInterfacePtr,
                                   const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr)
    : tbai::BobController(urdfString, robotInterfacePtr, refVelGenPtr) {
    if (!blind_) {
        gridmap_ = tbai::gridmap::getGridmapInterfaceUnique();
    }
    timeSinceLastVisualizationUpdate_ = 1000.0;
}

void RosBobController::postStep(scalar_t currentTime, scalar_t dt) {
    if (timeSinceLastVisualizationUpdate_ >= 1.0 / 30.0) {
        auto state = getBobnetState();
        stateVisualizer_.visualize(state);
        heightsReconstructedVisualizer_.visualize(state, sampled_, hidden_);
        contactVisualizer_.visualize(state_.x, state_.contactFlags);
        timeSinceLastVisualizationUpdate_ = 0.0;
    } else {
        timeSinceLastVisualizationUpdate_ += dt;
    }
}

void RosBobController::changeController(const std::string &controllerType, scalar_t currentTime) {
    preStep(currentTime, 0.0);
    if (!blind_) {
        gridmap_->waitTillInitialized();
    }
}

void RosBobController::atPositions(matrix_t &positions) {
    gridmap_->atPositions(positions);
}

void RosBobController::stopController() {
    // Implementation for stopController
}

bool RosBobController::ok() const {
    return ros::ok();
}

void RosBobController::preStep(scalar_t currentTime, scalar_t dt) {
    ros::spinOnce();
    state_ = robotInterfacePtr_->getLatestState();
}

}  // namespace rl
}  // namespace tbai
