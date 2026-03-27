#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <tuple>

#include <robot_state_publisher/robot_state_publisher.h>
#include <ros/ros.h>
#include <tbai_core/control/RobotInterface.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace tbai {

class StateVisualizer {
   public:
    StateVisualizer(std::shared_ptr<tbai::RobotInterface> interface, const std::vector<std::string> &jointNames,
                    const std::vector<std::string> &footFrameNames, double rate = 30.0, bool enableContactVis = true,
                    const std::string &baseName = "", const std::tuple<uint8_t, uint8_t, uint8_t> &contactColor = std::make_tuple(255, 0, 0));
    ~StateVisualizer();

   private:
    void publishJointTF(const State &state, const ros::Time &stamp);
    void publishContacts(const State &state, const ros::Time &stamp);

    std::shared_ptr<tbai::RobotInterface> interface_;
    std::vector<std::string> jointNames_;
    std::vector<std::string> footFrameNames_;
    std::string baseName_;
    bool enableContactVis_;
    std::tuple<float, float, float> contactColorNormalized_;
    ros::Publisher contactPublisher_;
    std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;
    std::atomic<bool> running_;
    std::thread thread_;
};

}  // namespace tbai
