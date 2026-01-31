#pragma once

#include <memory>
#include <string>

#include <tbai_deploy_g1/G1PBHCController.hpp>

namespace tbai {
namespace g1 {

/**
 * @brief ROS wrapper for G1PBHCController.
 *
 * Thin wrapper that adds ROS integration (ros::spinOnce, ros::ok)
 * while delegating all control logic to the base G1PBHCController.
 */
class RosG1PBHCController : public tbai::g1::G1PBHCController {
   public:
    /**
     * @brief Construct a new RosG1PBHCController
     * @param stateSubscriberPtr State subscriber for robot state
     * @param policyPath Path to ONNX model (.onnx file)
     * @param motionFilePath Path to motion CSV file (converted from pkl)
     * @param timeStart Start time in motion file
     * @param timeEnd End time in motion file (-1 for full duration)
     * @param controllerName Name for logging
     */
    RosG1PBHCController(const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr, const std::string &policyPath,
                        const std::string &motionFilePath, float timeStart = 0.0f, float timeEnd = -1.0f,
                        const std::string &controllerName = "RosG1PBHCController");

    void preStep(scalar_t currentTime, scalar_t dt) override;

    bool ok() const override;
};

}  // namespace g1
}  // namespace tbai
