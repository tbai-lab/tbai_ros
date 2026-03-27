#pragma once

#include <memory>
#include <string>

#include <tbai_deploy_g1/G1ASAPMimicController.hpp>

namespace tbai {
namespace g1 {

/**
 * @brief ROS wrapper for G1ASAPMimicController.
 *
 * Thin wrapper that adds ROS integration (ros::spinOnce, ros::ok)
 * while delegating all control logic to the base G1ASAPMimicController.
 */
class RosG1ASAPMimicController : public tbai::g1::G1ASAPMimicController {
   public:
    /**
     * @brief Construct a new RosG1ASAPMimicController
     * @param stateSubscriberPtr State subscriber for robot state
     * @param policyPath Path to ONNX model (.onnx file)
     * @param motionLength Duration of the motion in seconds
     * @param controllerName Name for logging
     */
    RosG1ASAPMimicController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                             const std::string &policyPath, float motionLength,
                             const std::string &controllerName = "G1ASAPMimic");

    void preStep(scalar_t currentTime, scalar_t dt) override;

    bool ok() const override;
};

}  // namespace g1
}  // namespace tbai
