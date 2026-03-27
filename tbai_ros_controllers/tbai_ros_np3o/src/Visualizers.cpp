#include "tbai_ros_np3o/Visualizers.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tbai_core/config/Config.hpp>
#include <urdf/model.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace tbai {

namespace np3o {

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
StateVisualizer::StateVisualizer() {
    // Load odom frame name
    odomFrame_ = tbai::fromGlobalConfig<std::string>("odom_frame");

    // Load base frame name
    baseFrame_ = tbai::fromGlobalConfig<std::string>("base_name");

    // Load joint names
    jointNames_ = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");

    // Setup state publisher
    std::string urdfString;
    if (!ros::param::get("/robot_description", urdfString)) {
        throw std::runtime_error("Failed to get param /robot_description");
    }

    KDL::Tree kdlTree;
    kdl_parser::treeFromString(urdfString, kdlTree);
    robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
    robotStatePublisherPtr_->publishFixedTransforms(true);

    ROS_INFO("StateVisualizer initialized");
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void StateVisualizer::visualize(const ::tbai::np3o::State &state) {
    ros::Time timeStamp = ros::Time::now();
    publishOdomTransform(timeStamp, state);
    publishJointAngles(timeStamp, state);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void StateVisualizer::publishOdomTransform(const ros::Time &timeStamp, const ::tbai::np3o::State &state) {
    geometry_msgs::TransformStamped baseToWorldTransform;
    baseToWorldTransform.header.stamp = timeStamp;
    baseToWorldTransform.header.frame_id = odomFrame_;
    baseToWorldTransform.child_frame_id = baseFrame_;

    baseToWorldTransform.transform.translation.x = state.basePositionWorld[0];
    baseToWorldTransform.transform.translation.y = state.basePositionWorld[1];
    baseToWorldTransform.transform.translation.z = state.basePositionWorld[2];

    baseToWorldTransform.transform.rotation.x = state.baseOrientationWorld[0];
    baseToWorldTransform.transform.rotation.y = state.baseOrientationWorld[1];
    baseToWorldTransform.transform.rotation.z = state.baseOrientationWorld[2];
    baseToWorldTransform.transform.rotation.w = state.baseOrientationWorld[3];

    tfBroadcaster_.sendTransform(baseToWorldTransform);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void StateVisualizer::publishJointAngles(const ros::Time &timeStamp, const ::tbai::np3o::State &state) {
    std::map<std::string, double> positions;
    for (int i = 0; i < jointNames_.size(); ++i) {
        positions[jointNames_[i]] = state.jointPositions[i];
    }
    robotStatePublisherPtr_->publishTransforms(positions, timeStamp);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
ContactVisualizer::ContactVisualizer() {
    ros::NodeHandle nh;
    contactPublisher_ = nh.advertise<visualization_msgs::MarkerArray>("/contact_points", 1);

    odomFrame_ = tbai::fromGlobalConfig<std::string>("odom_frame");
    footFrameNames_ = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};

    // Setup Pinocchio model
    std::string urdfString;
    if (!ros::param::get("/robot_description", urdfString)) {
        throw std::runtime_error("Failed to get param /robot_description");
    }

    pinocchio::urdf::buildModelFromXML(urdfString, pinocchio::JointModelFreeFlyer(), model_);
    data_ = pinocchio::Data(model_);
}

/***********************************************************************************************************************/
/***********************************************************************************************************************/
/***********************************************************************************************************************/
void ContactVisualizer::visualize(const vector_t &currentState, const std::vector<bool> &contacts) {
    ros::Time timeStamp = ros::Time::now();

    vector_t q = vector_t::Zero(model_.nq);
    q.head<3>() = currentState.segment<3>(3);                               // Position
    q.segment<4>(3) = tbai::ocs2rpy2quat(currentState.head<3>()).coeffs();  // Orientation
    q.tail(12) = currentState.segment<12>(3 + 3 + 3 + 3);
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);

    visualization_msgs::MarkerArray markerArray;
    for (size_t i = 0; i < footFrameNames_.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.stamp = timeStamp;
        marker.header.frame_id = odomFrame_;
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = contacts[i] ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
        marker.pose.position.x = data_.oMf[model_.getFrameId(footFrameNames_[i])].translation()[0];
        marker.pose.position.y = data_.oMf[model_.getFrameId(footFrameNames_[i])].translation()[1];
        marker.pose.position.z = data_.oMf[model_.getFrameId(footFrameNames_[i])].translation()[2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.07;
        marker.scale.y = 0.07;
        marker.scale.z = 0.07;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        markerArray.markers.push_back(marker);
    }
    contactPublisher_.publish(markerArray);
}

}  // namespace np3o
}  // namespace tbai
