// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <atomic>
#include <memory>
#include <thread>

#include "tbai_ros_mpc/quadruped_mpc/RosMpcController.hpp"
#include "tbai_ros_static/StaticController.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tbai_core/Env.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/CentralController.hpp>
#include <tbai_deploy_go2/Go2RobotInterface.hpp>
#include <tbai_ros_core/Rate.hpp>
#include <tbai_ros_core/Subscribers.hpp>
#include <tbai_ros_reference/ReferenceVelocityGenerator.hpp>
#include <tbai_sdk/messages/robot_msgs.hpp>
#include <tbai_sdk/subscriber.hpp>

struct CameraStream {
    std::string zenoh_topic;
    std::string ros_topic;
    std::string frame_id;
};

class SensorBridge {
   public:
    SensorBridge(bool publishImages, bool publishPointclouds) : running_(false) {
        if (!publishImages && !publishPointclouds) return;

        ros::NodeHandle nh;

        if (publishImages) {
            std::vector<CameraStream> cams = {
                {"rt/camera/image", "camera/color/image_raw", "camera_link_optical"},
            };
            for (const auto &cam : cams) {
                imgPubs_.push_back(nh.advertise<sensor_msgs::Image>(cam.ros_topic, 1));
                imgSubs_.push_back(std::make_unique<tbai::PollingSubscriber<robot_msgs::ImgFrame>>(cam.zenoh_topic));
                imgFrameIds_.push_back(cam.frame_id);
            }
        }

        if (publishPointclouds) {
            std::vector<CameraStream> pcs = {
                {"rt/pointcloud", "pointcloud/front", "front_camera_optical"},
            };
            for (const auto &pc : pcs) {
                pcPubs_.push_back(nh.advertise<sensor_msgs::PointCloud2>(pc.ros_topic, 1));
                pcSubs_.push_back(std::make_unique<tbai::PollingSubscriber<robot_msgs::PointCloud2>>(pc.zenoh_topic));
                pcFrameIds_.push_back(pc.frame_id);
            }
        }

        running_ = true;
        thread_ = std::thread([this]() {
            ros::Rate rate(30.0);
            while (ros::ok() && running_) {
                for (size_t i = 0; i < imgSubs_.size(); ++i) {
                    auto frame = imgSubs_[i]->take();
                    if (frame) {
                        sensor_msgs::Image msg;
                        msg.header.stamp = ros::Time::now();
                        msg.header.frame_id = imgFrameIds_[i];
                        msg.height = frame->height;
                        msg.width = frame->width;
                        msg.encoding = frame->encoding;
                        msg.is_bigendian = frame->is_bigendian;
                        msg.step = frame->step;
                        msg.data = std::move(frame->data);
                        imgPubs_[i].publish(msg);
                    }
                }
                for (size_t i = 0; i < pcSubs_.size(); ++i) {
                    auto pc = pcSubs_[i]->take();
                    if (pc) {
                        sensor_msgs::PointCloud2 msg;
                        msg.header.stamp = ros::Time::now();
                        msg.header.frame_id = pcFrameIds_[i];
                        msg.height = pc->height;
                        msg.width = pc->width;
                        msg.fields.resize(pc->fields.size());
                        for (size_t j = 0; j < pc->fields.size(); ++j) {
                            msg.fields[j].name = pc->fields[j].name;
                            msg.fields[j].offset = pc->fields[j].offset;
                            msg.fields[j].datatype = pc->fields[j].datatype;
                            msg.fields[j].count = pc->fields[j].count;
                        }
                        msg.is_bigendian = pc->is_bigendian;
                        msg.point_step = pc->point_step;
                        msg.row_step = pc->row_step;
                        msg.is_dense = pc->is_dense;
                        msg.data = std::move(pc->data);
                        pcPubs_[i].publish(msg);
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
    std::vector<ros::Publisher> imgPubs_, pcPubs_;
    std::vector<std::unique_ptr<tbai::PollingSubscriber<robot_msgs::ImgFrame>>> imgSubs_;
    std::vector<std::unique_ptr<tbai::PollingSubscriber<robot_msgs::PointCloud2>>> pcSubs_;
    std::vector<std::string> imgFrameIds_, pcFrameIds_;
    std::atomic<bool> running_;
    std::thread thread_;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "tbai_ros_go2_mpc_mujoco");
    ros::NodeHandle nh;

    // Set zero time
    tbai::writeInitTime(tbai::RosTime::rightNow());

    // Sensor bridge for cameras and pointclouds
    bool publishImages = tbai::getEnvAs<bool>("TBAI_GO2_PUBLISH_IMAGE", true);
    bool publishPointclouds = tbai::getEnvAs<bool>("TBAI_GO2_PUBLISH_POINTCLOUD", true);
    auto sensorBridge = std::make_unique<SensorBridge>(publishImages, publishPointclouds);

    // Initialize Go2RobotInterface
    tbai::Go2RobotInterfaceArgs go2Args;
    go2Args.useGroundTruthState = tbai::getEnvAs<bool>("TBAI_GO2_USE_GROUND_TRUTH", false);
    auto go2RobotInterface = std::make_shared<tbai::Go2RobotInterface>(go2Args);

    auto changeControllerTopic = tbai::fromGlobalConfig<std::string>("change_controller_topic");

    std::shared_ptr<tbai::ChangeControllerSubscriber> changeControllerSubscriber =
        std::shared_ptr<tbai::ChangeControllerSubscriber>(
            new tbai::RosChangeControllerSubscriber(nh, changeControllerTopic));

    tbai::CentralController<ros::Rate, tbai::RosTime> controller(go2RobotInterface, changeControllerSubscriber);

    // Add static controller
    controller.addController(std::make_unique<tbai::static_::RosStaticController>(go2RobotInterface));

    std::string urdfString = nh.param<std::string>("robot_description", "");

    if (urdfString.empty()) {
        throw std::runtime_error("Failed to get param /robot_description");
    }

    auto referenceVelocityPtr = tbai::reference::getReferenceVelocityGeneratorShared(nh);

    // Add MPC controller
    std::string robotName = tbai::fromGlobalConfig<std::string>("robot_name");
    controller.addController(std::make_unique<tbai::mpc::RosMpcController>(
        robotName, go2RobotInterface, referenceVelocityPtr, tbai::RosTime::rightNow));

    // Start controller loop
    controller.start();

    return EXIT_SUCCESS;
}
