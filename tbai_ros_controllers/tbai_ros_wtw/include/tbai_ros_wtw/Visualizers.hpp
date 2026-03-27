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
#include <tbai_wtw/State.hpp>
#include <tf/transform_broadcaster.h>
#include <torch/script.h>

namespace tbai {
namespace wtw {

using namespace torch::indexing;  // NOLINT

class StateVisualizer {
   public:
    StateVisualizer();
    void visualize(const ::tbai::wtw::State &state);

   private:
    void publishOdomTransform(const ros::Time &timeStamp, const ::tbai::wtw::State &state);
    void publishJointAngles(const ros::Time &timeStamp, const ::tbai::wtw::State &state);

    std::string odomFrame_;
    std::string baseFrame_;
    std::vector<std::string> jointNames_;

    std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;
    tf::TransformBroadcaster tfBroadcaster_;
};

class ContactVisualizer {
   public:
    ContactVisualizer();
    void visualize(const vector_t &currentState, const std::vector<bool> &contacts);

   private:
    std::string odomFrame_;
    ros::Publisher contactPublisher_;
    std::vector<std::string> footFrameNames_;

    pinocchio::Model model_;
    pinocchio::Data data_;
};

}  // namespace wtw
}  // namespace tbai
