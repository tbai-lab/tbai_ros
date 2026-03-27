// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <atomic>
#include <memory>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tbai_core/Env.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/CentralController.hpp>
#include <tbai_deploy_spot/SpotRobotInterface.hpp>
#include <tbai_ros_core/Rate.hpp>
#include <tbai_ros_core/StateVisualizer.hpp>
#include <tbai_ros_core/Subscribers.hpp>
#include <tbai_ros_mpc/quadruped_mpc/RosMpcController.hpp>
#include <tbai_ros_reference/ReferenceVelocityGenerator.hpp>
#include <tbai_ros_static/StaticController.hpp>
#include <tbai_sdk/messages/robot_msgs.hpp>
#include <tbai_sdk/subscriber.hpp>

struct CameraStream {
    std::string zenoh_topic;
    std::string ros_topic;
    std::string frame_id;
};

class SensorBridge {
   public:
    SensorBridge(bool publishImage) : running_(false) {
        if (!publishImage) return;

        std::vector<CameraStream> cameras = {
            {"rt/camera/front", "camera/front/image_raw", "camera_front"},
            {"rt/camera/front_left", "camera/front_left/image_raw", "camera_front_left"},
            {"rt/camera/front_right", "camera/front_right/image_raw", "camera_front_right"},
            {"rt/camera/left", "camera/left/image_raw", "camera_left"},
            {"rt/camera/right", "camera/right/image_raw", "camera_right"},
            {"rt/camera/back", "camera/back/image_raw", "camera_back"},
        };

        ros::NodeHandle nh;
        for (const auto &cam : cameras) {
            publishers_.push_back(nh.advertise<sensor_msgs::Image>(cam.ros_topic, 1));
            subscribers_.push_back(std::make_unique<tbai::PollingSubscriber<robot_msgs::ImgFrame>>(cam.zenoh_topic));
            frame_ids_.push_back(cam.frame_id);
        }

        running_ = true;
        thread_ = std::thread([this]() {
            ros::Rate rate(30.0);
            while (ros::ok() && running_) {
                for (size_t i = 0; i < subscribers_.size(); ++i) {
                    auto frame = subscribers_[i]->take();
                    if (frame) {
                        sensor_msgs::Image msg;
                        msg.header.stamp = ros::Time::now();
                        msg.header.frame_id = frame_ids_[i];
                        msg.height = frame->height;
                        msg.width = frame->width;
                        msg.encoding = frame->encoding;
                        msg.is_bigendian = frame->is_bigendian;
                        msg.step = frame->step;
                        msg.data = std::move(frame->data);
                        publishers_[i].publish(msg);
                    }
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
    std::vector<ros::Publisher> publishers_;
    std::vector<std::unique_ptr<tbai::PollingSubscriber<robot_msgs::ImgFrame>>> subscribers_;
    std::vector<std::string> frame_ids_;
    std::atomic<bool> running_;
    std::thread thread_;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "deploy_spot");
    ros::NodeHandle nh;

    auto logger = tbai::getLogger("deploy_spot");
    TBAI_LOG_INFO(logger, "Starting Spot deployment node");

    std::string robotName = tbai::fromGlobalConfig<std::string>("robot_name");

    // Set zero time
    tbai::writeInitTime(tbai::RosTime::rightNow());

    // Start sensor bridge for mujoco camera
    bool publishImage = tbai::getEnvAs<bool>("TBAI_SPOT_PUBLISH_IMAGE", true, false);
    auto sensorBridge = std::make_unique<SensorBridge>(publishImage);

    // SpotRobotInterface is both StateSubscriber and CommandPublisher
    auto spotInterface = std::make_shared<tbai::SpotRobotInterface>(tbai::SpotRobotInterfaceArgs());

    // State visualizer
    auto jointNames = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");
    std::vector<std::string> footFrames = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
    auto stateVisualizer =
        std::make_unique<tbai::StateVisualizer>(spotInterface, jointNames, footFrames, 30.0, false, "base");

    std::shared_ptr<tbai::StateSubscriber> stateSubscriber = spotInterface;
    std::shared_ptr<tbai::CommandPublisher> commandPublisher = spotInterface;

    auto changeControllerTopic = tbai::fromGlobalConfig<std::string>("change_controller_topic");
    auto changeControllerSubscriber = std::make_shared<tbai::RosChangeControllerSubscriber>(nh, changeControllerTopic);

    tbai::CentralController<ros::Rate, tbai::RosTime> controller(commandPublisher, changeControllerSubscriber);

    // Add static controller
    controller.addController(std::make_unique<tbai::static_::RosStaticController>(stateSubscriber));

    // Add MPC controller
    controller.addController(std::make_unique<tbai::mpc::RosMpcController>(
        robotName, stateSubscriber, tbai::reference::getReferenceVelocityGeneratorShared(nh), tbai::RosTime::rightNow));

    TBAI_LOG_INFO(logger, "Controllers initialized. Starting main loop...");

    controller.start();

    return EXIT_SUCCESS;
}
