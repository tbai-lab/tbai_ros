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
#include <tbai_bob/BobState.hpp>
#include <tbai_core/Rotations.hpp>
#include <tf/transform_broadcaster.h>
#include <torch/script.h>

namespace tbai {
namespace rl {

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
    void visualize(const BobState &state);

   private:
    /** Publish odom->base transform */
    void publishOdomTransform(const ros::Time &timeStamp, const BobState &state);

    /** Publish joint angles via a robot_state_publisher */
    void publishJointAngles(const ros::Time &timeStamp, const BobState &state);

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

class HeightsReconstructedVisualizer {
   public:
    /**
     * @brief Construct a new Heights Reconstructed Visualizer object
     *
     */
    HeightsReconstructedVisualizer();

    /**
     * @brief Visualize sampled and reconstructed height samples
     *
     * @param state : Current state
     * @param sampled : 3xN matrix, where each column is a point in 3D space
     * @param nnPointsReconstructed : reconstructed tensor from the neural network
     */
    void visualize(const BobState &state, const matrix_t &sampled, const at::Tensor &nnPointsReconstructed);

   private:
    /** Publish 3d points as RViz markers */
    void publishMarkers(const ros::Time &timeStamp, const matrix_t &sampled, const std::array<float, 3> &rgb,
                        const std::string &markerNamePrefix, std::function<scalar_t(size_t)> heightFunction);

    /** Odom frame name */
    std::string odomFrame_;

    /** ROS publisher for markers */
    ros::Publisher markerPublisher_;

    /** Whether or not the robot is blind */
    bool blind_;
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

}  // namespace rl
}  // namespace tbai
