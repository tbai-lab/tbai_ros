#include <ros/ros.h>
#include <tbai_ros_g1_mujoco/G1Twist2Controller.hpp>

namespace tbai {
namespace g1 {

RosG1Twist2Controller::RosG1Twist2Controller(const std::shared_ptr<tbai::RobotInterface> &robotInterfacePtr,
                                             const std::string &policyPath, const std::string &motionFilePath,
                                             float timeStart, float timeEnd, const std::string &controllerName)
    : G1Twist2Controller(robotInterfacePtr, policyPath, motionFilePath, timeStart, timeEnd, controllerName) {}

void RosG1Twist2Controller::preStep(scalar_t currentTime, scalar_t dt) {
    ros::spinOnce();
    G1Twist2Controller::preStep(currentTime, dt);
}

bool RosG1Twist2Controller::ok() const {
    return ros::ok();
}

}  // namespace g1
}  // namespace tbai
