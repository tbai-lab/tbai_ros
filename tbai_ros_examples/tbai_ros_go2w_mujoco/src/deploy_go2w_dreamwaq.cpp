#include <atomic>
#include <iostream>
#include <memory>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tbai_core/Env.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/CentralController.hpp>
#include <tbai_deploy_go2w/Go2WRobotInterface.hpp>
#include <tbai_ros_core/Rate.hpp>
#include <tbai_ros_core/StateVisualizer.hpp>
#include <tbai_ros_core/Subscribers.hpp>
#include <tbai_ros_go2w_mujoco/Go2WDreamWaQController.hpp>
#include <tbai_ros_reference/ReferenceVelocityGenerator.hpp>
#include <tbai_ros_static/StaticController.hpp>
#include <tbai_sdk/messages/robot_msgs.hpp>
#include <tbai_sdk/subscriber.hpp>

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
                    msg.header.frame_id = "head_camera";
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
    ros::init(argc, argv, "tbai_ros_deploy_go2w_dreamwaq");
    ros::NodeHandle nh;

    auto logger = tbai::getLogger("deploy_go2w_dreamwaq");
    TBAI_LOG_INFO(logger, "Starting Go2W DreamWaQ deployment node");

    // Set zero time
    tbai::writeInitTime(tbai::RosTime::rightNow());

    // Start sensor bridge
    bool publishImage = tbai::getEnvAs<bool>("TBAI_GO2W_PUBLISH_IMAGE", true);
    auto sensorBridge = std::make_unique<SensorBridge>(publishImage);

    // Initialize Go2WRobotInterface
    tbai::Go2WRobotInterfaceArgs ifaceArgs;
    ifaceArgs.useGroundTruthState(tbai::getEnvAs<bool>("TBAI_GO2W_USE_GROUND_TRUTH", false));
    std::shared_ptr<tbai::Go2WRobotInterface> go2wRobotInterface =
        std::make_shared<tbai::Go2WRobotInterface>(ifaceArgs);

    // State visualizer (TF + contacts)
    auto jointNames = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");
    std::vector<std::string> footFrames = {"FR_foot", "FL_foot", "RR_foot", "RL_foot"};
    auto stateVisualizer =
        std::make_unique<tbai::StateVisualizer>(go2wRobotInterface, jointNames, footFrames, 30.0, false, "base");

    auto changeControllerTopic = tbai::fromGlobalConfig<std::string>("change_controller_topic");

    std::shared_ptr<tbai::ChangeControllerSubscriber> changeControllerSubscriber =
        std::make_shared<tbai::RosChangeControllerSubscriber>(nh, changeControllerTopic);

    // Create central controller
    tbai::CentralController<ros::Rate, tbai::RosTime> controller(go2wRobotInterface, changeControllerSubscriber);

    // Add static controller for standing
    controller.addController(std::make_unique<tbai::static_::RosStaticController>(go2wRobotInterface));

    // Get reference velocity generator
    auto referenceGeneratorType = tbai::fromGlobalConfig<std::string>("reference_generator/type");
    std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> referenceVelocityPtr;

    if (referenceGeneratorType == "twist") {
        referenceVelocityPtr = tbai::reference::getReferenceVelocityGeneratorUnique(nh);
    } else if (referenceGeneratorType == "joystick") {
        referenceVelocityPtr = tbai::reference::getReferenceVelocityGeneratorUnique(nh);
    } else {
        TBAI_LOG_WARN(logger, "Unknown reference generator type: {}, using twist", referenceGeneratorType);
        referenceVelocityPtr = tbai::reference::getReferenceVelocityGeneratorUnique(nh);
    }

    auto hfRepo = tbai::fromGlobalConfig<std::string>("go2w_controller/hf_repo");
    auto hfModelFolder = tbai::fromGlobalConfig<std::string>("go2w_controller/hf_model_folder");
    auto modelDir = tbai::downloadFromHuggingFace(hfRepo, hfModelFolder);
    TBAI_LOG_INFO(logger, "Loading HF model: {}/{}", hfRepo, hfModelFolder);

    // Add Go2W DreamWaQ controller
    controller.addController(
        std::make_unique<tbai::go2w::RosGo2WDreamWaQController>(go2wRobotInterface, referenceVelocityPtr, modelDir));

    TBAI_LOG_INFO(logger, "Controllers initialized. Starting main loop...");

    // Start controller loop
    controller.start();

    return EXIT_SUCCESS;
}
