// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <atomic>
#include <cstring>
#include <memory>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tbai_core/Env.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/CentralController.hpp>
#include <tbai_deploy_franka/FrankaRobotInterface.hpp>
#include <tbai_ros_core/Rate.hpp>
#include <tbai_ros_core/Subscribers.hpp>
#include <tbai_ros_mpc/arm_mpc/ArmMpcController.hpp>
#include <tbai_ros_static/StaticController.hpp>
#include <tbai_ros_core/StateVisualizer.hpp>
#include <tbai_sdk/subscriber.hpp>
#include <tbai_sdk/messages/robot_msgs.hpp>

// Sensor bridge: subscribes to tbai_sdk zenoh topics and republishes as ROS messages
class SensorBridge {
   public:
    SensorBridge(bool publishImage) : running_(false) {
        if (!publishImage) return;

        ros::NodeHandle nh;
        imagePublisher_ = nh.advertise<sensor_msgs::Image>("camera/color/image_raw", 1);
        imageSubscriber_ = std::make_unique<tbai::PollingSubscriber<robot_msgs::ImgFrame>>("rt/camera/image");

        running_ = true;
        thread_ = std::thread([this]() {
            ros::Rate rate(30.0);
            while (ros::ok() && running_) {
                auto frame = imageSubscriber_->take();
                if (frame) {
                    sensor_msgs::Image msg;
                    msg.header.stamp = ros::Time::now();
                    msg.header.frame_id = "wrist_camera";
                    msg.height = frame->height;
                    msg.width = frame->width;
                    msg.encoding = frame->encoding;
                    msg.is_bigendian = frame->is_bigendian;
                    msg.step = frame->step;
                    msg.data = std::move(frame->data);
                    imagePublisher_.publish(msg);
                }
                rate.sleep();
            }
        });
    }

    ~SensorBridge() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

   private:
    ros::Publisher imagePublisher_;
    std::unique_ptr<tbai::PollingSubscriber<robot_msgs::ImgFrame>> imageSubscriber_;
    std::atomic<bool> running_;
    std::thread thread_;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "arm_wbc");
    ros::NodeHandle nh;

    // Set zero time
    tbai::writeInitTime(tbai::RosTime::rightNow());

    // Start sensor bridge for mujoco camera
    bool publishImage = tbai::getEnvAs<bool>("TBAI_FRANKA_PUBLISH_IMAGE", true, false);
    auto sensorBridge = std::make_unique<SensorBridge>(publishImage);

    // The FrankaRobotInterface is both StateSubscriber and CommandPublisher
    // (replaces the ROS topic-based state/command used in the simulation version)
    auto frankaInterface = std::make_shared<tbai::FrankaRobotInterface>(tbai::FrankaRobotInterfaceArgs());

    // No StateVisualizer for Franka — ArmMpcController's ArmVisualizer handles TF

    std::shared_ptr<tbai::StateSubscriber> stateSubscriber = frankaInterface;
    std::shared_ptr<tbai::CommandPublisher> commandPublisher = frankaInterface;

    auto changeControllerTopic = tbai::fromGlobalConfig<std::string>("change_controller_topic");
    auto changeControllerSubscriber =
        std::make_shared<tbai::RosChangeControllerSubscriber>(nh, changeControllerTopic);

    tbai::CentralController<ros::Rate, tbai::RosTime> controller(commandPublisher, changeControllerSubscriber);

    // Add static controller (starts first, holds home position)
    controller.addController(std::make_unique<tbai::static_::RosStaticController>(stateSubscriber));

    // Add Arm MPC+WBC controller (switch to this via change_controller_topic)
    controller.addController(
        std::make_unique<tbai::mpc::arm::ArmMpcController>(stateSubscriber, tbai::RosTime::rightNow));

    // Start controller loop
    controller.start();

    return EXIT_SUCCESS;
}
