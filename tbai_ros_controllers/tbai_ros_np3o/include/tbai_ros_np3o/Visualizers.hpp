#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <memory>
#include <string>
#include <vector>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <robot_state_publisher/robot_state_publisher.h>
#include <ros/ros.h>
#include <tbai_core/Rotations.hpp>
#include <tbai_np3o/State.hpp>
#include <tf/transform_broadcaster.h>
#include <torch/script.h>

namespace tbai {
namespace np3o {

using namespace torch::indexing;  // NOLINT

class StateVisualizer {
   public:
    /**
     * @brief Construct a new State Visualizer object
     *
     */
    StateVisualizer();

    /**
     * @brief Publish odom->base transform and joint angles
     *
     * @param state : Current state
     */
    void visualize(const ::tbai::np3o::State &state);

   private:
    /** Publish odom->base transform */
    void publishOdomTransform(const ros::Time &timeStamp, const ::tbai::np3o::State &state);

    /** Publish joint angles via a robot_state_publisher */
    void publishJointAngles(const ros::Time &timeStamp, const ::tbai::np3o::State &state);

    /** Odom frame name */
    std::string odomFrame_;

    /** Base frame name */
    std::string baseFrame_;

    /** List of joint names - must be in the same order as in State*/
    std::vector<std::string> jointNames_;

    /** Helper classes for state visualization */
    std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;
    tf::TransformBroadcaster tfBroadcaster_;
};

class ContactVisualizer {
   public:
    ContactVisualizer();
    void visualize(const vector_t &currentState, const std::vector<bool> &contacts);

   private:
    /** Odom frame name */
    std::string odomFrame_;

    /** Base frame name */
    ros::Publisher contactPublisher_;

    /** List of foot frame names */
    std::vector<std::string> footFrameNames_;

    pinocchio::Model model_;
    pinocchio::Data data_;
};

}  // namespace np3o
}  // namespace tbai
