#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_state_publisher/robot_state_publisher.h>
#include <ros/ros.h>
#include <tbai_core/Rotations.hpp>
#include <tbai_core/Types.hpp>
#include <tbai_core/control/Subscribers.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

namespace tbai {

class StateVisualizer {
   public:
    StateVisualizer(std::shared_ptr<tbai::StateSubscriber> interface, const std::vector<std::string> &jointNames,
                    const std::vector<std::string> &footFrameNames, double rate = 30.0, bool enableContactVis = true,
                    const std::string &baseName = "")
        : interface_(interface),
          jointNames_(jointNames),
          footFrameNames_(footFrameNames),
          baseName_(baseName),
          enableContactVis_(enableContactVis),
          running_(true) {
        ros::NodeHandle nh;
        if (enableContactVis_ && !footFrameNames_.empty()) {
            contactPublisher_ = nh.advertise<visualization_msgs::MarkerArray>("/contact_points", 1);
        }

        std::string urdfString;
        ros::param::get("/robot_description", urdfString);
        if (!urdfString.empty()) {
            KDL::Tree kdlTree;
            if (kdl_parser::treeFromString(urdfString, kdlTree)) {
                robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
                robotStatePublisherPtr_->publishFixedTransforms(true);
            }
            // pinocchio model not needed — contacts use TF frames directly
        }

        thread_ = std::thread([this, rate]() {
            ros::Rate r(rate);
            while (ros::ok() && running_) {
                auto state = interface_->getLatestState();
                if (state.x.size() > 0) {
                    auto now = ros::Time::now();
                    publishJointTF(state, now);
                    if (enableContactVis_ && !footFrameNames_.empty()) {
                        publishContacts(state, now);
                    }
                }
                r.sleep();
            }
        });
    }

    ~StateVisualizer() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

   private:
    void publishJointTF(const State &state, const ros::Time &stamp) {
        if (!robotStatePublisherPtr_) return;

        // Publish odom -> base transform
        if (!baseName_.empty()) {
            const auto quat = tbai::ocs2rpy2quat(state.x.head<3>());
            geometry_msgs::TransformStamped baseTransform;
            baseTransform.header.stamp = stamp;
            baseTransform.header.frame_id = "odom";
            baseTransform.child_frame_id = baseName_;
            baseTransform.transform.translation.x = state.x[3];
            baseTransform.transform.translation.y = state.x[4];
            baseTransform.transform.translation.z = state.x[5];
            baseTransform.transform.rotation.x = quat.x();
            baseTransform.transform.rotation.y = quat.y();
            baseTransform.transform.rotation.z = quat.z();
            baseTransform.transform.rotation.w = quat.w();
            tfBroadcaster_.sendTransform(baseTransform);
        }

        // Publish joint transforms
        std::map<std::string, double> positions;
        for (size_t i = 0; i < jointNames_.size(); ++i) {
            if (12 + i < static_cast<size_t>(state.x.size())) {
                positions[jointNames_[i]] = state.x[12 + i];
            }
        }
        robotStatePublisherPtr_->publishTransforms(positions, stamp);
    }

    void publishContacts(const State &state, const ros::Time &stamp) {
        visualization_msgs::MarkerArray markerArray;
        for (size_t i = 0; i < footFrameNames_.size() && i < state.contactFlags.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.header.stamp = stamp;
            marker.header.frame_id = footFrameNames_[i];  // Relative to foot TF frame
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action =
                state.contactFlags[i] ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
            marker.pose.position.x = 0.0;
            marker.pose.position.y = 0.0;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.05;
            marker.color.r = 1.0;
            marker.color.a = 1.0;
            markerArray.markers.push_back(marker);
        }
        contactPublisher_.publish(markerArray);
    }

    std::shared_ptr<tbai::StateSubscriber> interface_;
    std::vector<std::string> jointNames_;
    std::vector<std::string> footFrameNames_;
    std::string baseName_;
    bool enableContactVis_;
    ros::Publisher contactPublisher_;
    std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;
    std::atomic<bool> running_;
    std::thread thread_;
};

}  // namespace tbai
