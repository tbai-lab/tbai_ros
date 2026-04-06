#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <memory>
#include <string>
#include <vector>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <robot_state_publisher/robot_state_publisher.h>
#include <ros/ros.h>
#include <tbai_core/control/Controllers.hpp>
#include <tbai_static/StaticController.hpp>
#include <tf/transform_broadcaster.h>

namespace tbai {
namespace static_ {

class RosStaticController : public tbai::static_::StaticController {
   public:
    /**
     * @brief Construct a new StaticController object
     *
     */
    RosStaticController(std::shared_ptr<tbai::RobotInterface> robotInterfacePtr);
    void postStep(scalar_t currentTime, scalar_t dt) override;

    bool ok() const override { return ros::ok(); }

    void preStep(scalar_t currentTime, scalar_t dt) override {
        ros::spinOnce();
        state_ = robotInterfacePtr_->getLatestState();
    }

   private:
    /** Publish odom->base transforms */
    void publishOdomBaseTransforms(const vector_t &currentState, const ros::Time &currentTime);

    /** Publish joint angles */
    void publishJointAngles(const vector_t &currentState, const ros::Time &currentTime);

    /** Setup Pinocchio model */
    void setupPinocchioModel();

    /** Visualize contact points */
    void visualizeContactPoints(const vector_t &currentState, const std::vector<bool> &contacts,
                                const ros::Time &currentTime);

    /** Visualization */
    tf::TransformBroadcaster tfBroadcaster_;
    std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;

    std::string baseFrameName_;

    /** Time since last visualization step */
    scalar_t timeSinceLastVisualizationUpdate_;

    pinocchio::Model model_;
    pinocchio::Data data_;
    ros::Publisher contactPointPublisher_;
    std::vector<std::string> footFrameNames_;
};

}  // namespace static_
}  // namespace tbai
