// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <unistd.h>

#include <atomic>
#include <memory>
#include <thread>

#include <execinfo.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tbai_core/Env.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/CentralController.hpp>
#include <tbai_deploy_spot/SpotArmRobotInterface.hpp>
#include <tbai_ros_core/Rate.hpp>
#include <tbai_ros_core/StateVisualizer.hpp>
#include <tbai_ros_core/Subscribers.hpp>
#include <tbai_ros_mpc/quadruped_arm_mpc/RosMpcController.hpp>
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
                    msg.header.frame_id = "camera_front";
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
    ros::init(argc, argv, "deploy_spot_arm");
    ros::NodeHandle nh;

    auto logger = tbai::getLogger("deploy_spot_arm");
    TBAI_LOG_INFO(logger, "Starting Spot Arm deployment node");

    try {
        std::string robotName = tbai::fromGlobalConfig<std::string>("robot_name");
        tbai::writeInitTime(tbai::RosTime::rightNow());

        bool publishImage = tbai::getEnvAs<bool>("TBAI_SPOT_PUBLISH_IMAGE", false);
        auto sensorBridge = std::make_unique<SensorBridge>(publishImage);

        auto spotArmInterface = std::make_shared<tbai::SpotArmRobotInterface>(tbai::SpotArmRobotInterfaceArgs());

        auto jointNames = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");
        std::vector<std::string> footFrames = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
        auto stateVisualizer =
            std::make_unique<tbai::StateVisualizer>(spotArmInterface, jointNames, footFrames, 30.0, false, "base");

        auto changeControllerTopic = tbai::fromGlobalConfig<std::string>("change_controller_topic");
        auto changeControllerSubscriber =
            std::make_shared<tbai::RosChangeControllerSubscriber>(nh, changeControllerTopic);

        tbai::CentralController<ros::Rate, tbai::RosTime> controller(spotArmInterface, changeControllerSubscriber);

        controller.addController(std::make_unique<tbai::static_::RosStaticController>(spotArmInterface));

        controller.addController(std::make_unique<tbai::mpc::RosMpcController>(
            robotName, spotArmInterface, tbai::reference::getReferenceVelocityGeneratorShared(nh),
            tbai::RosTime::rightNow));

        TBAI_LOG_INFO(logger, "Controllers initialized. Starting main loop...");
        controller.start();

    } catch (const std::exception &e) {
        std::cerr << "FATAL: " << e.what() << std::endl;
        // Print backtrace
        void *array[20];
        int size = backtrace(array, 20);
        backtrace_symbols_fd(array, size, STDERR_FILENO);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
