#pragma once

#include <memory>
#include <string>

#include <tbai_deploy_g1/G1ASAPController.hpp>

namespace tbai {
namespace g1 {

/**
 * @brief ROS wrapper for G1ASAPController.
 *
 * Thin wrapper that adds ROS integration (ros::spinOnce, ros::ok)
 * while delegating all control logic to the base G1ASAPController.
 */
class RosG1ASAPController : public tbai::g1::G1ASAPController {
   public:
    /**
     * @brief Construct a new RosG1ASAPController
     * @param stateSubscriberPtr State subscriber for robot state
     * @param refVelGenPtr Reference velocity generator for velocity commands
     * @param policyPath Path to ONNX model (.onnx file)
     * @param controllerName Name for logging
     */
    RosG1ASAPController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                        const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr,
                        const std::string &policyPath, const std::string &controllerName = "G1ASAPLocomotion");

    void preStep(scalar_t currentTime, scalar_t dt) override;

    bool ok() const override;
};

}  // namespace g1
}  // namespace tbai
