// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <atomic>
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
#include <tbai_deploy_anymal_d/AnymalDRobotInterface.hpp>
#include <tbai_ros_core/Rate.hpp>
#include <tbai_ros_core/StateVisualizer.hpp>
#include <tbai_ros_core/Subscribers.hpp>
#include <tbai_ros_joe/RosJoeController.hpp>
#include <tbai_ros_reference/ReferenceVelocityGenerator.hpp>
#include <tbai_ros_static/StaticController.hpp>
#include <tbai_sdk/messages/robot_msgs.hpp>
#include <tbai_sdk/subscriber.hpp>

struct CameraStream {
    std::string zenoh_topic, ros_topic, frame_id;
};

class SensorBridge {
   public:
    SensorBridge(bool publishImages, bool publishPointclouds) : running_(false) {
        if (!publishImages && !publishPointclouds) return;
        ros::NodeHandle nh;
        if (publishImages) {
            std::vector<CameraStream> cams = {
                {"rt/camera/front", "camera/front/image_raw", "wide_angle_camera_front_camera"},
                {"rt/camera/rear", "camera/rear/image_raw", "wide_angle_camera_rear_camera"},
            };
            for (const auto &c : cams) {
                imgPubs_.push_back(nh.advertise<sensor_msgs::Image>(c.ros_topic, 1));
                imgSubs_.push_back(std::make_unique<tbai::PollingSubscriber<robot_msgs::ImgFrame>>(c.zenoh_topic));
                imgFrameIds_.push_back(c.frame_id);
            }
        }
        if (publishPointclouds) {
            std::vector<CameraStream> pcs = {
                {"rt/pointcloud/front_upper", "pointcloud/front_upper", "depth_camera_front_upper_depth_optical_frame"},
                {"rt/pointcloud/front_lower", "pointcloud/front_lower", "depth_camera_front_lower_depth_optical_frame"},
                {"rt/pointcloud/rear_upper", "pointcloud/rear_upper", "depth_camera_rear_upper_depth_optical_frame"},
                {"rt/pointcloud/rear_lower", "pointcloud/rear_lower", "depth_camera_rear_lower_depth_optical_frame"},
                {"rt/pointcloud/left", "pointcloud/left", "depth_camera_left_depth_optical_frame"},
                {"rt/pointcloud/right", "pointcloud/right", "depth_camera_right_depth_optical_frame"},
            };
            for (const auto &p : pcs) {
                pcPubs_.push_back(nh.advertise<sensor_msgs::PointCloud2>(p.ros_topic, 1));
                pcSubs_.push_back(std::make_unique<tbai::PollingSubscriber<robot_msgs::PointCloud2>>(p.zenoh_topic));
                pcFrameIds_.push_back(p.frame_id);
            }
        }
        running_ = true;
        thread_ = std::thread([this]() {
            ros::Rate rate(30.0);
            while (ros::ok() && running_) {
                for (size_t i = 0; i < imgSubs_.size(); ++i) {
                    auto f = imgSubs_[i]->take();
                    if (f) {
                        sensor_msgs::Image m;
                        m.header.stamp = ros::Time::now();
                        m.header.frame_id = imgFrameIds_[i];
                        m.height = f->height;
                        m.width = f->width;
                        m.encoding = f->encoding;
                        m.is_bigendian = f->is_bigendian;
                        m.step = f->step;
                        m.data = std::move(f->data);
                        imgPubs_[i].publish(m);
                    }
                }
                for (size_t i = 0; i < pcSubs_.size(); ++i) {
                    auto p = pcSubs_[i]->take();
                    if (p) {
                        sensor_msgs::PointCloud2 m;
                        m.header.stamp = ros::Time::now();
                        m.header.frame_id = pcFrameIds_[i];
                        m.height = p->height;
                        m.width = p->width;
                        m.fields.resize(p->fields.size());
                        for (size_t j = 0; j < p->fields.size(); ++j) {
                            m.fields[j].name = p->fields[j].name;
                            m.fields[j].offset = p->fields[j].offset;
                            m.fields[j].datatype = p->fields[j].datatype;
                            m.fields[j].count = p->fields[j].count;
                        }
                        m.is_bigendian = p->is_bigendian;
                        m.point_step = p->point_step;
                        m.row_step = p->row_step;
                        m.is_dense = p->is_dense;
                        m.data = std::move(p->data);
                        pcPubs_[i].publish(m);
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
    ros::init(argc, argv, "deploy_anymal_d_joe");
    ros::NodeHandle nh;

    auto logger = tbai::getLogger("deploy_anymal_d_joe");
    TBAI_LOG_INFO(logger, "Starting ANYmal D JOE deployment node");

    auto robotName = tbai::fromGlobalConfig<std::string>("robot_name");
    tbai::writeInitTime(tbai::RosTime::rightNow());

    bool publishImages = tbai::getEnvAs<bool>("TBAI_ANYMAL_PUBLISH_IMAGE", false);
    bool publishPointclouds = tbai::getEnvAs<bool>("TBAI_ANYMAL_PUBLISH_POINTCLOUD", false);
    auto sensorBridge = std::make_unique<SensorBridge>(publishImages, publishPointclouds);

    tbai::AnymalDRobotInterfaceArgs ifaceArgs;
    ifaceArgs.useGroundTruthState = true;
    ifaceArgs.enableGroundPlaneCorrection = false;
    auto anymalInterface = std::make_shared<tbai::AnymalDRobotInterface>(ifaceArgs);

    auto jointNames = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");
    std::vector<std::string> footFrames = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
    auto stateVisualizer =
        std::make_unique<tbai::StateVisualizer>(anymalInterface, jointNames, footFrames, 30.0, false, "base");

    auto changeControllerTopic = tbai::fromGlobalConfig<std::string>("change_controller_topic");
    auto changeControllerSubscriber = std::make_shared<tbai::RosChangeControllerSubscriber>(nh, changeControllerTopic);

    tbai::CentralController<ros::Rate, tbai::RosTime> controller(anymalInterface, changeControllerSubscriber);

    controller.addController(std::make_unique<tbai::static_::RosStaticController>(anymalInterface));

    controller.addController(std::make_unique<tbai::joe::RosJoeController>(
        robotName, anymalInterface, tbai::reference::getReferenceVelocityGeneratorShared(nh), tbai::RosTime::rightNow));

    TBAI_LOG_INFO(logger, "Controllers initialized. Starting main loop...");
    controller.start();

    return EXIT_SUCCESS;
}
