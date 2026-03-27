#include <atomic>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <tbai_core/Env.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/CentralController.hpp>
#include <tbai_deploy_g1/G1RobotInterface.hpp>
#include <tbai_ros_core/Rate.hpp>
#include <tbai_ros_core/StateVisualizer.hpp>
#include <tbai_ros_core/Subscribers.hpp>
#include <tbai_ros_g1_mujoco/G1ASAPController.hpp>
#include <tbai_ros_g1_mujoco/G1ASAPMimicController.hpp>
#include <tbai_ros_g1_mujoco/G1BeyondMimicController.hpp>
#include <tbai_ros_g1_mujoco/G1MimicController.hpp>
#include <tbai_ros_g1_mujoco/G1PBHCController.hpp>
#include <tbai_ros_g1_mujoco/G1RLController.hpp>
#include <tbai_ros_g1_mujoco/G1SpinkickController.hpp>
#include <tbai_ros_g1_mujoco/G1Twist2Controller.hpp>
#include <tbai_ros_reference/ReferenceVelocityGenerator.hpp>
#include <tbai_ros_static/StaticController.hpp>
#include <tbai_sdk/messages/robot_msgs.hpp>
#include <tbai_sdk/subscriber.hpp>

// Sensor bridge: subscribes to tbai_sdk zenoh topics and republishes as ROS messages
class SensorBridge {
   public:
    SensorBridge(bool publishImage, bool publishPointcloud) : running_(false) {
        ros::NodeHandle nh;

        if (publishImage) {
            imagePublisher_ = nh.advertise<sensor_msgs::Image>("camera/color/image_raw", 1);
            imageSubscriber_ = std::make_unique<tbai::PollingSubscriber<robot_msgs::ImgFrame>>("rt/camera/image");
            ROS_INFO("G1 SensorBridge: subscribed to rt/camera/image");
        }

        if (publishPointcloud) {
            pointcloudPublisher_ = nh.advertise<sensor_msgs::PointCloud2>("camera/depth/points", 1);
            pointcloudSubscriber_ = std::make_unique<tbai::PollingSubscriber<robot_msgs::PointCloud2>>("rt/pointcloud");
            ROS_INFO("G1 SensorBridge: subscribed to rt/pointcloud");
        }

        if (publishImage || publishPointcloud) {
            running_ = true;
            thread_ = std::thread([this, publishImage, publishPointcloud]() {
                ros::Rate rate(30.0);
                while (ros::ok() && running_) {
                    if (publishImage) publishColorImage();
                    if (publishPointcloud) publishDepthPointcloud();
                    rate.sleep();
                }
            });
        }
    }

    ~SensorBridge() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

   private:
    void publishColorImage() {
        if (!imageSubscriber_) return;
        auto frame = imageSubscriber_->take();
        if (!frame) return;

        sensor_msgs::Image img_msg;
        img_msg.header.stamp = ros::Time::now();
        img_msg.header.frame_id = "d435_optical";
        img_msg.height = frame->height;
        img_msg.width = frame->width;
        img_msg.encoding = frame->encoding;
        img_msg.is_bigendian = frame->is_bigendian;
        img_msg.step = frame->step;
        img_msg.data = std::move(frame->data);

        imagePublisher_.publish(img_msg);
    }

    void publishDepthPointcloud() {
        if (!pointcloudSubscriber_) return;
        auto pc = pointcloudSubscriber_->take();
        if (!pc) return;

        sensor_msgs::PointCloud2 pc2_msg;
        pc2_msg.header.stamp = ros::Time::now();
        pc2_msg.header.frame_id = "d435_optical";
        pc2_msg.height = pc->height;
        pc2_msg.width = pc->width;

        pc2_msg.fields.resize(pc->fields.size());
        for (size_t i = 0; i < pc->fields.size(); ++i) {
            pc2_msg.fields[i].name = pc->fields[i].name;
            pc2_msg.fields[i].offset = pc->fields[i].offset;
            pc2_msg.fields[i].datatype = pc->fields[i].datatype;
            pc2_msg.fields[i].count = pc->fields[i].count;
        }

        pc2_msg.is_bigendian = pc->is_bigendian;
        pc2_msg.point_step = pc->point_step;
        pc2_msg.row_step = pc->row_step;
        pc2_msg.is_dense = pc->is_dense;
        pc2_msg.data = std::move(pc->data);

        pointcloudPublisher_.publish(pc2_msg);
    }

    ros::Publisher imagePublisher_;
    ros::Publisher pointcloudPublisher_;
    std::unique_ptr<tbai::PollingSubscriber<robot_msgs::ImgFrame>> imageSubscriber_;
    std::unique_ptr<tbai::PollingSubscriber<robot_msgs::PointCloud2>> pointcloudSubscriber_;
    std::atomic<bool> running_;
    std::thread thread_;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "tbai_ros_deploy_g1_rl");
    ros::NodeHandle nh;

    auto logger = tbai::getLogger("deploy_g1_rl");
    TBAI_LOG_INFO(logger, "Starting G1 RL deployment node");

    // Set zero time
    tbai::writeInitTime(tbai::RosTime::rightNow());

    // Start sensor bridge for mujoco image/pointcloud publishing
    bool publishImage = tbai::getEnvAs<bool>("TBAI_G1_PUBLISH_IMAGE", false);
    bool publishPointcloud = tbai::getEnvAs<bool>("TBAI_G1_PUBLISH_POINTCLOUD", false);
    auto sensorBridge = std::make_unique<SensorBridge>(publishImage, publishPointcloud);

    // Initialize G1RobotInterface
    std::shared_ptr<tbai::G1RobotInterface> g1RobotInterface = std::make_shared<tbai::G1RobotInterface>(
        tbai::G1RobotInterfaceArgs()
            .networkInterface(tbai::getEnvAs<std::string>("TBAI_G1_NETWORK_INTERFACE", "lo"))
            .unitreeChannel(tbai::getEnvAs<int>("TBAI_G1_UNITREE_CHANNEL", 1))
            .useGroundTruthState(tbai::getEnvAs<bool>("TBAI_G1_USE_GROUND_TRUTH", false)));

    // Start state visualizer (TF + contact markers) for all controllers
    auto jointNames = tbai::fromGlobalConfig<std::vector<std::string>>("joint_names");
    std::vector<std::string> footFrames = {"left_ankle_roll_link", "right_ankle_roll_link"};
    auto stateVisualizer =
        std::make_unique<tbai::StateVisualizer>(g1RobotInterface, jointNames, footFrames, 30.0, true, "pelvis");

    auto changeControllerTopic = tbai::fromGlobalConfig<std::string>("change_controller_topic");

    std::shared_ptr<tbai::ChangeControllerSubscriber> changeControllerSubscriber =
        std::make_shared<tbai::RosChangeControllerSubscriber>(nh, changeControllerTopic);

    tbai::CentralController<ros::Rate, tbai::RosTime> controller(g1RobotInterface, changeControllerSubscriber);

    // Add static controller for standing
    controller.addController(std::make_unique<tbai::static_::RosStaticController>(g1RobotInterface));

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

    // Helper: download model, construct controller, add with lock
    std::mutex controllerMutex;
    auto addCtrl = [&](std::shared_ptr<tbai::Controller> ctrl) {
        std::lock_guard<std::mutex> lock(controllerMutex);
        controller.addController(std::move(ctrl));
    };

    std::vector<std::thread> loaderThreads;

    // G1 RL walking controller
    loaderThreads.emplace_back([&]() {
        auto repo = tbai::fromGlobalConfig<std::string>("g1_controller/hf_repo");
        auto model = tbai::fromGlobalConfig<std::string>("g1_controller/hf_model");
        auto path = tbai::downloadFromHuggingFace(repo, model);
        TBAI_LOG_INFO(logger, "Loaded G1RLController: {}/{}", repo, model);
        addCtrl(std::make_shared<tbai::g1::RosG1RLController>(g1RobotInterface, referenceVelocityPtr, path));
    });

    // Mimic dance 102
    loaderThreads.emplace_back([&]() {
        auto repo = tbai::fromGlobalConfig<std::string>("g1_mimic_dance102/hf_repo");
        auto model = tbai::fromGlobalConfig<std::string>("g1_mimic_dance102/hf_model");
        auto fps = tbai::fromGlobalConfig<float>("g1_mimic_dance102/motion_fps");
        auto ts = tbai::fromGlobalConfig<float>("g1_mimic_dance102/time_start");
        auto te = tbai::fromGlobalConfig<float>("g1_mimic_dance102/time_end");
        auto motionFile = tbai::fromGlobalConfig<std::string>("g1_mimic_dance102/hf_motion_file");
        auto modelPath = tbai::downloadFromHuggingFace(repo, model);
        auto motionPath = tbai::downloadFromHuggingFace(repo, motionFile);
        TBAI_LOG_INFO(logger, "Loaded G1MimicDance102: {}/{}", repo, model);
        addCtrl(std::make_shared<tbai::g1::RosG1MimicController>(g1RobotInterface, modelPath, motionPath, fps, ts, te,
                                                                 "G1MimicDance102"));
    });

    // Mimic gangnam
    loaderThreads.emplace_back([&]() {
        auto repo = tbai::fromGlobalConfig<std::string>("g1_mimic_gangnam/hf_repo");
        auto model = tbai::fromGlobalConfig<std::string>("g1_mimic_gangnam/hf_model");
        auto fps = tbai::fromGlobalConfig<float>("g1_mimic_gangnam/motion_fps");
        auto ts = tbai::fromGlobalConfig<float>("g1_mimic_gangnam/time_start");
        auto te = tbai::fromGlobalConfig<float>("g1_mimic_gangnam/time_end");
        auto motionFile = tbai::fromGlobalConfig<std::string>("g1_mimic_gangnam/hf_motion_file");
        auto modelPath = tbai::downloadFromHuggingFace(repo, model);
        auto motionPath = tbai::downloadFromHuggingFace(repo, motionFile);
        TBAI_LOG_INFO(logger, "Loaded G1MimicGangnam: {}/{}", repo, model);
        addCtrl(std::make_shared<tbai::g1::RosG1MimicController>(g1RobotInterface, modelPath, motionPath, fps, ts, te,
                                                                 "G1MimicGangnam"));
    });

    // BeyondMimic Dance
    loaderThreads.emplace_back([&]() {
        auto repo = tbai::fromGlobalConfig<std::string>("g1_beyond_dance/hf_repo");
        auto model = tbai::fromGlobalConfig<std::string>("g1_beyond_dance/hf_model");
        auto path = tbai::downloadFromHuggingFace(repo, model);
        TBAI_LOG_INFO(logger, "Loaded G1BeyondDance: {}", path);
        addCtrl(std::make_shared<tbai::g1::RosG1BeyondMimicController>(g1RobotInterface, path, "G1BeyondDance"));
    });

    // Spinkick
    loaderThreads.emplace_back([&]() {
        auto repo = tbai::fromGlobalConfig<std::string>("g1_spinkick/hf_repo");
        auto model = tbai::fromGlobalConfig<std::string>("g1_spinkick/hf_model");
        auto path = tbai::downloadFromHuggingFace(repo, model);
        TBAI_LOG_INFO(logger, "Loaded G1Spinkick: {}", path);
        addCtrl(std::make_shared<tbai::g1::RosG1SpinkickController>(g1RobotInterface, path, "G1Spinkick"));
    });

    // TWIST2 controllers (shared model, different motions)
    auto twistHfRepo = tbai::fromGlobalConfig<std::string>("g1_twist_walk1/hf_repo");
    auto twistHfModel = tbai::fromGlobalConfig<std::string>("g1_twist_walk1/hf_model");
    auto twistModelPath = tbai::downloadFromHuggingFace(twistHfRepo, twistHfModel);

    auto loadTwist = [&](const std::string &configPrefix, const std::string &ctrlName) {
        loaderThreads.emplace_back([&, configPrefix, ctrlName, twistModelPath]() {
            auto motionFile = tbai::fromGlobalConfig<std::string>(configPrefix + "/hf_motion_file");
            auto motionPath = tbai::downloadFromHuggingFace(twistHfRepo, motionFile);
            auto ts = tbai::fromGlobalConfig<float>(configPrefix + "/time_start");
            auto te = tbai::fromGlobalConfig<float>(configPrefix + "/time_end");
            TBAI_LOG_INFO(logger, "Loaded {}: {}", ctrlName, motionFile);
            addCtrl(std::make_shared<tbai::g1::RosG1Twist2Controller>(g1RobotInterface, twistModelPath, motionPath, ts,
                                                                      te, ctrlName));
        });
    };
    loadTwist("g1_twist_walk1", "G1TwistWalk1");
    loadTwist("g1_twist_walk2", "G1TwistWalk2");
    loadTwist("g1_twist_walk3", "G1TwistWalk3");
    loadTwist("g1_twist_walk5", "G1TwistWalk5");
    loadTwist("g1_twist_walk7", "G1TwistWalk7");
    loadTwist("g1_twist_swing", "G1TwistSwing");

    // PBHC controllers
    auto loadPBHC = [&](const std::string &configPrefix, const std::string &ctrlName) {
        loaderThreads.emplace_back([&, configPrefix, ctrlName]() {
            auto repo = tbai::fromGlobalConfig<std::string>(configPrefix + "/hf_repo");
            auto model = tbai::fromGlobalConfig<std::string>(configPrefix + "/hf_model");
            auto motionFile = tbai::fromGlobalConfig<std::string>(configPrefix + "/hf_motion_file");
            auto modelPath = tbai::downloadFromHuggingFace(repo, model);
            auto motionPath = tbai::downloadFromHuggingFace(repo, motionFile);
            auto ts = tbai::fromGlobalConfig<float>(configPrefix + "/time_start");
            auto te = tbai::fromGlobalConfig<float>(configPrefix + "/time_end");
            TBAI_LOG_INFO(logger, "Loaded {}: {}/{}", ctrlName, repo, model);
            addCtrl(std::make_shared<tbai::g1::RosG1PBHCController>(g1RobotInterface, modelPath, motionPath, ts, te,
                                                                    ctrlName));
        });
    };
    loadPBHC("g1_pbhc_horse_stance_punch", "G1PBHCHorseStancePunch");
    loadPBHC("g1_pbhc_horse_stance_pose", "G1PBHCHorseStancePose");
    loadPBHC("g1_pbhc_horse_stance_pose2", "G1PBHCHorseStancePose2");

    // ASAP Locomotion
    loaderThreads.emplace_back([&]() {
        auto repo = tbai::fromGlobalConfig<std::string>("g1_asap_locomotion/hf_repo");
        auto model = tbai::fromGlobalConfig<std::string>("g1_asap_locomotion/hf_model");
        auto path = tbai::downloadFromHuggingFace(repo, model);
        TBAI_LOG_INFO(logger, "Loaded G1ASAPLocomotion: {}/{}", repo, model);
        addCtrl(std::make_shared<tbai::g1::RosG1ASAPController>(g1RobotInterface, referenceVelocityPtr, path,
                                                                "G1ASAPLocomotion"));
    });

    // ASAP Mimic controllers
    auto loadASAPMimic = [&](const std::string &configPrefix, const std::string &ctrlName) {
        loaderThreads.emplace_back([&, configPrefix, ctrlName]() {
            auto repo = tbai::fromGlobalConfig<std::string>(configPrefix + "/hf_repo");
            auto model = tbai::fromGlobalConfig<std::string>(configPrefix + "/hf_model");
            auto path = tbai::downloadFromHuggingFace(repo, model);
            auto motionLength = tbai::fromGlobalConfig<float>(configPrefix + "/motion_length");
            TBAI_LOG_INFO(logger, "Loaded {}: {}/{}", ctrlName, repo, model);
            addCtrl(
                std::make_shared<tbai::g1::RosG1ASAPMimicController>(g1RobotInterface, path, motionLength, ctrlName));
        });
    };
    loadASAPMimic("g1_asap_cr7", "G1ASAPCR7");
    loadASAPMimic("g1_asap_apt", "G1ASAPAPT");
    loadASAPMimic("g1_asap_jump_forward1", "G1ASAPJumpForward1");
    loadASAPMimic("g1_asap_jump_forward2", "G1ASAPJumpForward2");
    loadASAPMimic("g1_asap_jump_forward3", "G1ASAPJumpForward3");
    loadASAPMimic("g1_asap_kick1", "G1ASAPKick1");
    loadASAPMimic("g1_asap_kick2", "G1ASAPKick2");
    loadASAPMimic("g1_asap_kick3", "G1ASAPKick3");
    loadASAPMimic("g1_asap_kobe", "G1ASAPKobe");
    loadASAPMimic("g1_asap_lebron1", "G1ASAPLeBron1");
    loadASAPMimic("g1_asap_lebron2", "G1ASAPLeBron2");
    loadASAPMimic("g1_asap_side_jump1", "G1ASAPSideJump1");
    loadASAPMimic("g1_asap_side_jump2", "G1ASAPSideJump2");
    loadASAPMimic("g1_asap_side_jump3", "G1ASAPSideJump3");

    // Wait for all loader threads to finish
    for (auto &t : loaderThreads) {
        t.join();
    }

    // Remove threads
    loaderThreads.clear();

    TBAI_LOG_INFO(logger, "Controllers initialized. Starting main loop...");

    // Start controller loop
    controller.start();

    return EXIT_SUCCESS;
}
