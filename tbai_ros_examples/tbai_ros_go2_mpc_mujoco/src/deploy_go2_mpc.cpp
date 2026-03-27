// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <atomic>
#include <cstring>
#include <memory>
#include <thread>

#include "tbai_ros_mpc/quadruped_mpc/RosMpcController.hpp"
#include "tbai_ros_static/StaticController.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <tbai_core/Env.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/CentralController.hpp>
#include <tbai_deploy_go2/Go2RobotInterface.hpp>
#include <tbai_ros_core/Publishers.hpp>
#include <tbai_ros_core/Rate.hpp>
#include <tbai_ros_core/Subscribers.hpp>
#include <tbai_ros_go2_mpc_mujoco/Go2Joystick.hpp>
#include <tbai_ros_reference/ReferenceVelocityGenerator.hpp>
#include <tbai_sdk/messages/robot_msgs.hpp>
#include <tbai_sdk/subscriber.hpp>

class Go2RobotInterfaceWithSensors : public tbai::Go2RobotInterface {
   public:
    Go2RobotInterfaceWithSensors(const tbai::Go2RobotInterfaceArgs &args)
        : tbai::Go2RobotInterface(args), publishSensors_(false) {
        ros::NodeHandle nh;

        if (args.subscribeLidar()) {
            lidarPublisher_ = nh.advertise<sensor_msgs::PointCloud2>("unitree_lidar_points", 1);
        }

        bool publishPointcloud = args.subscribePointcloud();
        bool publishImage = args.enableVideo();

        if (publishPointcloud) {
            depthPointcloudPublisher_ = nh.advertise<sensor_msgs::PointCloud2>("camera/depth/points", 1);
        }

        if (publishImage) {
            imagePublisher_ = nh.advertise<sensor_msgs::Image>("camera/color/image_raw", 1);
            std::string imageTopic = "rt/camera/image";
            imageSubscriber_ = std::make_unique<tbai::PollingSubscriber<robot_msgs::ImgFrame>>(imageTopic);
            ROS_INFO("Subscribed to tbai_sdk image topic: %s", imageTopic.c_str());
        }

        if (publishPointcloud || publishImage) {
            publishSensors_ = true;
            sensorPublishThread_ = std::thread([this, publishPointcloud, publishImage]() {
                ros::Rate rate(30.0);
                while (ros::ok() && publishSensors_) {
                    if (publishPointcloud) publishDepthPointcloud();
                    if (publishImage) publishColorImage();
                    rate.sleep();
                }
            });
        }
    }

    ~Go2RobotInterfaceWithSensors() {
        publishSensors_ = false;
        if (sensorPublishThread_.joinable()) {
            sensorPublishThread_.join();
        }
    }

    void lidarCallback(const robot_msgs::PointCloud2 &msg) override {
        sensor_msgs::PointCloud2 pc2_msg;

        pc2_msg.header.stamp = ros::Time(msg.header.stamp.sec, msg.header.stamp.nanosec);
        pc2_msg.header.frame_id = "radar";
        pc2_msg.height = msg.height;
        pc2_msg.width = msg.width;

        pc2_msg.fields.resize(msg.fields.size());
        for (size_t i = 0; i < msg.fields.size(); ++i) {
            const auto &field = msg.fields[i];
            pc2_msg.fields[i].name = field.name;
            pc2_msg.fields[i].offset = field.offset;
            pc2_msg.fields[i].datatype = field.datatype;
            pc2_msg.fields[i].count = field.count;
        }

        pc2_msg.is_bigendian = msg.is_bigendian;
        pc2_msg.point_step = msg.point_step;
        pc2_msg.row_step = msg.row_step;
        pc2_msg.is_dense = msg.is_dense;
        pc2_msg.data = msg.data;

        lidarPublisher_.publish(pc2_msg);
    }

   private:
    void publishDepthPointcloud() {
        auto points = getLatestPointcloud();
        if (points.empty()) return;

        uint32_t num_points = points.size() / 3;

        sensor_msgs::PointCloud2 pc2_msg;
        pc2_msg.header.stamp = ros::Time::now();
        pc2_msg.header.frame_id = "camera_link_optical";
        pc2_msg.height = 1;
        pc2_msg.width = num_points;

        sensor_msgs::PointField field;
        field.count = 1;
        field.datatype = sensor_msgs::PointField::FLOAT32;

        field.name = "x";
        field.offset = 0;
        pc2_msg.fields.push_back(field);
        field.name = "y";
        field.offset = 4;
        pc2_msg.fields.push_back(field);
        field.name = "z";
        field.offset = 8;
        pc2_msg.fields.push_back(field);

        pc2_msg.is_bigendian = false;
        pc2_msg.point_step = 12;
        pc2_msg.row_step = pc2_msg.point_step * num_points;
        pc2_msg.is_dense = true;

        pc2_msg.data.resize(points.size() * sizeof(float));
        std::memcpy(pc2_msg.data.data(), points.data(), pc2_msg.data.size());

        depthPointcloudPublisher_.publish(pc2_msg);
    }

    void publishColorImage() {
        if (!imageSubscriber_) return;
        auto frame = imageSubscriber_->take();
        if (!frame) return;

        sensor_msgs::Image img_msg;
        img_msg.header.stamp = ros::Time::now();
        img_msg.header.frame_id = "camera_link_optical";
        img_msg.height = frame->height;
        img_msg.width = frame->width;
        img_msg.encoding = frame->encoding;
        img_msg.is_bigendian = frame->is_bigendian;
        img_msg.step = frame->step;
        img_msg.data = std::move(frame->data);

        imagePublisher_.publish(img_msg);
    }

    ros::Publisher lidarPublisher_;
    ros::Publisher depthPointcloudPublisher_;
    ros::Publisher imagePublisher_;
    std::unique_ptr<tbai::PollingSubscriber<robot_msgs::ImgFrame>> imageSubscriber_;
    std::atomic<bool> publishSensors_;
    std::thread sensorPublishThread_;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "tbai_ros_go2_mpc_mujoco");
    ros::NodeHandle nh;

    // Set zero time
    tbai::writeInitTime(tbai::RosTime::rightNow());

    // Initialize Go2RobotInterface
    std::shared_ptr<tbai::Go2RobotInterface> go2RobotInterface =
        std::shared_ptr<tbai::Go2RobotInterface>(new Go2RobotInterfaceWithSensors(
            tbai::Go2RobotInterfaceArgs()
                .networkInterface(tbai::getEnvAs<std::string>("TBAI_GO2_NETWORK_INTERFACE", true, "eth0"))
                .subscribeLidar(tbai::getEnvAs<bool>("TBAI_GO2_PUBLISH_LIDAR", true, false))
                .unitreeChannel(tbai::getEnvAs<int>("TBAI_GO2_UNITREE_CHANNEL", true, 0))
                .subscribePointcloud(tbai::getEnvAs<bool>("TBAI_GO2_SUBSCRIBE_POINTCLOUD", true, true))
                .enableVideo(tbai::getEnvAs<bool>("TBAI_GO2_ENABLE_VIDEO", true, true))
                .useGroundTruthState(tbai::getEnvAs<bool>("TBAI_GO2_USE_GROUND_TRUTH", true, false))));

    std::shared_ptr<tbai::StateSubscriber> stateSubscriber = go2RobotInterface;
    std::shared_ptr<tbai::CommandPublisher> commandPublisher = go2RobotInterface;

    auto changeControllerTopic = tbai::fromGlobalConfig<std::string>("change_controller_topic");

    std::shared_ptr<tbai::ChangeControllerSubscriber> changeControllerSubscriber =
        std::shared_ptr<tbai::ChangeControllerSubscriber>(
            new tbai::RosChangeControllerSubscriber(nh, changeControllerTopic));

    tbai::CentralController<ros::Rate, tbai::RosTime> controller(commandPublisher, changeControllerSubscriber);

    // Add static controller
    controller.addController(std::make_unique<tbai::static_::RosStaticController>(stateSubscriber));

    std::string urdfString = nh.param<std::string>("robot_description", "");

    if (urdfString.empty()) {
        throw std::runtime_error("Failed to get param /robot_description");
    }

    auto referenceGeneratorType = tbai::fromGlobalConfig<std::string>("reference_generator/type");

    std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> referenceVelocityPtr;
    if (referenceGeneratorType == "go2_joystick") {
        auto joystick = tbai::reference::getGo2JoystickShared(nh);
        joystick->Start();
        referenceVelocityPtr = joystick;
    } else if (referenceGeneratorType == "joystick") {
        referenceVelocityPtr = tbai::reference::getReferenceVelocityGeneratorUnique(nh);
    } else if (referenceGeneratorType == "twist") {
        referenceVelocityPtr = tbai::reference::getReferenceVelocityGeneratorUnique(nh);
    } else {
        TBAI_THROW("Invalid reference generator type: {}. Supported types are: go2_joystick, joystick, twist",
                   referenceGeneratorType);
    }

    // Add MPC controller
    std::string robotName = tbai::fromGlobalConfig<std::string>("robot_name");
    controller.addController(std::make_unique<tbai::mpc::RosMpcController>(
        robotName, stateSubscriber, referenceVelocityPtr, tbai::RosTime::rightNow));

    // Start controller loop
    controller.start();

    return EXIT_SUCCESS;
}
