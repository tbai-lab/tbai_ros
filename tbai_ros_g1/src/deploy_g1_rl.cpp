#include <iostream>
#include <memory>

#include <ros/package.h>
#include <ros/ros.h>
#include <tbai_core/Env.hpp>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/CentralController.hpp>
#include <tbai_deploy_g1/G1RobotInterface.hpp>
#include <tbai_ros_core/Rate.hpp>
#include <tbai_ros_g1/G1ASAPController.hpp>
#include <tbai_ros_g1/G1BeyondMimicController.hpp>
#include <tbai_ros_g1/G1MimicController.hpp>
#include <tbai_ros_g1/G1PBHCController.hpp>
#include <tbai_ros_g1/G1RLController.hpp>
#include <tbai_ros_g1/G1SpinkickController.hpp>
#include <tbai_ros_g1/G1TwistController.hpp>
#include <tbai_ros_reference/ReferenceVelocityGenerator.hpp>
#include <tbai_ros_static/StaticController.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "tbai_ros_deploy_g1_rl");
    ros::NodeHandle nh;

    auto logger = tbai::getLogger("deploy_g1_rl");
    TBAI_LOG_INFO(logger, "Starting G1 RL deployment node");

    // Set zero time
    tbai::writeInitTime(tbai::RosTime::rightNow());

    // Initialize G1RobotInterface
    std::shared_ptr<tbai::G1RobotInterface> g1RobotInterface = std::make_shared<tbai::G1RobotInterface>(
        tbai::G1RobotInterfaceArgs()
            .networkInterface(tbai::getEnvAs<std::string>("TBAI_G1_NETWORK_INTERFACE", true, "lo"))
            .unitreeChannel(tbai::getEnvAs<int>("TBAI_G1_UNITREE_CHANNEL", true, 1)));

    std::shared_ptr<tbai::StateSubscriber> stateSubscriber = g1RobotInterface;
    std::shared_ptr<tbai::CommandPublisher> commandPublisher = g1RobotInterface;

    auto changeControllerTopic = tbai::fromGlobalConfig<std::string>("change_controller_topic");

    std::shared_ptr<tbai::ChangeControllerSubscriber> changeControllerSubscriber =
        std::make_shared<tbai::RosChangeControllerSubscriber>(nh, changeControllerTopic);

    tbai::CentralController<ros::Rate, tbai::RosTime> controller(commandPublisher, changeControllerSubscriber);

    // Add static controller for standing
    controller.addController(std::make_unique<tbai::static_::RosStaticController>(stateSubscriber));

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

    // Load G1 walking controller
    auto hfRepo = tbai::fromGlobalConfig<std::string>("g1_controller/hf_repo");
    auto hfModel = tbai::fromGlobalConfig<std::string>("g1_controller/hf_model");
    auto modelPath = tbai::downloadFromHuggingFace(hfRepo, hfModel);
    TBAI_LOG_INFO(logger, "Loading HF model: {}/{}", hfRepo, hfModel);
    controller.addController(
        std::make_unique<tbai::g1::RosG1RLController>(stateSubscriber, referenceVelocityPtr, modelPath));

    // Load G1 mimic dance 102 controller
    auto hfRepo3 = tbai::fromGlobalConfig<std::string>("g1_mimic_dance102/hf_repo");
    auto hfModel3 = tbai::fromGlobalConfig<std::string>("g1_mimic_dance102/hf_model");
    auto motionFps = tbai::fromGlobalConfig<float>("g1_mimic_dance102/motion_fps");
    auto timeStart = tbai::fromGlobalConfig<float>("g1_mimic_dance102/time_start");
    auto timeEnd = tbai::fromGlobalConfig<float>("g1_mimic_dance102/time_end");
    auto dance102MotionFile = tbai::fromGlobalConfig<std::string>("g1_mimic_dance102/hf_motion_file");
    auto modelPathDance102 = tbai::downloadFromHuggingFace(hfRepo3, hfModel3);
    auto dance102MotionFilePath = tbai::downloadFromHuggingFace(hfRepo3, dance102MotionFile);
    TBAI_LOG_INFO(logger, "Loading HF model: {}/{}", hfRepo3, hfModel3);
    controller.addController(std::make_unique<tbai::g1::RosG1MimicController>(
        stateSubscriber, modelPathDance102, dance102MotionFilePath, motionFps, timeStart, timeEnd, "G1MimicDance102"));

    // Load G1 mimic gangnam style controller
    auto hfRepo2 = tbai::fromGlobalConfig<std::string>("g1_mimic_gangnam/hf_repo");
    auto hfModel2 = tbai::fromGlobalConfig<std::string>("g1_mimic_gangnam/hf_model");
    auto motionFps2 = tbai::fromGlobalConfig<float>("g1_mimic_gangnam/motion_fps");
    auto timeStart2 = tbai::fromGlobalConfig<float>("g1_mimic_gangnam/time_start");
    auto timeEnd2 = tbai::fromGlobalConfig<float>("g1_mimic_gangnam/time_end");
    auto gangnamMotionFile2 = tbai::fromGlobalConfig<std::string>("g1_mimic_gangnam/hf_motion_file");
    auto modelPathGangnam = tbai::downloadFromHuggingFace(hfRepo2, hfModel2);
    auto gangnamMotionFilePath = tbai::downloadFromHuggingFace(hfRepo2, gangnamMotionFile2);
    TBAI_LOG_INFO(logger, "Loading HF model: {}/{}", hfRepo2, hfModel2);
    controller.addController(std::make_unique<tbai::g1::RosG1MimicController>(
        stateSubscriber, modelPathGangnam, gangnamMotionFilePath, motionFps2, timeStart2, timeEnd2, "G1MimicGangnam"));

    // Load G1 BeyondMimic Dance controller
    auto hfRepoBeyondDance = tbai::fromGlobalConfig<std::string>("g1_beyond_dance/hf_repo");
    auto hfModelBeyondDance = tbai::fromGlobalConfig<std::string>("g1_beyond_dance/hf_model");
    auto modelPathBeyondDance = tbai::downloadFromHuggingFace(hfRepoBeyondDance, hfModelBeyondDance);
    TBAI_LOG_INFO(logger, "Loading BeyondMimic model: {}", modelPathBeyondDance);
    controller.addController(
        std::make_unique<tbai::g1::RosG1BeyondMimicController>(stateSubscriber, modelPathBeyondDance, "G1BeyondDance"));

    // Load G1 Spinkick controller
    auto hfRepoSpinkick = tbai::fromGlobalConfig<std::string>("g1_spinkick/hf_repo");
    auto hfModelSpinkick = tbai::fromGlobalConfig<std::string>("g1_spinkick/hf_model");
    auto modelPathSpinkick = tbai::downloadFromHuggingFace(hfRepoSpinkick, hfModelSpinkick);
    TBAI_LOG_INFO(logger, "Loading Spinkick model: {}", modelPathSpinkick);
    controller.addController(
        std::make_unique<tbai::g1::RosG1SpinkickController>(stateSubscriber, modelPathSpinkick, "G1Spinkick"));

    // Load G1 TWIST2 controllers (ONNX model from amazon-far/TWIST2)
    // Walk motion 1
    auto twistModelPath = tbai::fromGlobalConfig<std::string>("g1_twist_walk1/model_path");
    auto twistWalk1Motion = tbai::fromGlobalConfig<std::string>("g1_twist_walk1/motion_file");
    auto twistWalk1Start = tbai::fromGlobalConfig<float>("g1_twist_walk1/time_start");
    auto twistWalk1End = tbai::fromGlobalConfig<float>("g1_twist_walk1/time_end");
    TBAI_LOG_INFO(logger, "Loading TWIST2 Walk1: {}", twistWalk1Motion);
    controller.addController(std::make_unique<tbai::g1::RosG1TwistController>(
        stateSubscriber, twistModelPath, twistWalk1Motion, twistWalk1Start, twistWalk1End, "G1TwistWalk1"));

    // Walk motion 2
    auto twistWalk2Motion = tbai::fromGlobalConfig<std::string>("g1_twist_walk2/motion_file");
    auto twistWalk2Start = tbai::fromGlobalConfig<float>("g1_twist_walk2/time_start");
    auto twistWalk2End = tbai::fromGlobalConfig<float>("g1_twist_walk2/time_end");
    TBAI_LOG_INFO(logger, "Loading TWIST2 Walk2: {}", twistWalk2Motion);
    controller.addController(std::make_unique<tbai::g1::RosG1TwistController>(
        stateSubscriber, twistModelPath, twistWalk2Motion, twistWalk2Start, twistWalk2End, "G1TwistWalk2"));

    // Walk motion 3
    auto twistWalk3Motion = tbai::fromGlobalConfig<std::string>("g1_twist_walk3/motion_file");
    auto twistWalk3Start = tbai::fromGlobalConfig<float>("g1_twist_walk3/time_start");
    auto twistWalk3End = tbai::fromGlobalConfig<float>("g1_twist_walk3/time_end");
    TBAI_LOG_INFO(logger, "Loading TWIST2 Walk3: {}", twistWalk3Motion);
    controller.addController(std::make_unique<tbai::g1::RosG1TwistController>(
        stateSubscriber, twistModelPath, twistWalk3Motion, twistWalk3Start, twistWalk3End, "G1TwistWalk3"));

    // Walk motion 5
    auto twistWalk5Motion = tbai::fromGlobalConfig<std::string>("g1_twist_walk5/motion_file");
    auto twistWalk5Start = tbai::fromGlobalConfig<float>("g1_twist_walk5/time_start");
    auto twistWalk5End = tbai::fromGlobalConfig<float>("g1_twist_walk5/time_end");
    TBAI_LOG_INFO(logger, "Loading TWIST2 Walk5: {}", twistWalk5Motion);
    controller.addController(std::make_unique<tbai::g1::RosG1TwistController>(
        stateSubscriber, twistModelPath, twistWalk5Motion, twistWalk5Start, twistWalk5End, "G1TwistWalk5"));

    // Walk motion 7
    auto twistWalk7Motion = tbai::fromGlobalConfig<std::string>("g1_twist_walk7/motion_file");
    auto twistWalk7Start = tbai::fromGlobalConfig<float>("g1_twist_walk7/time_start");
    auto twistWalk7End = tbai::fromGlobalConfig<float>("g1_twist_walk7/time_end");
    TBAI_LOG_INFO(logger, "Loading TWIST2 Walk7: {}", twistWalk7Motion);
    controller.addController(std::make_unique<tbai::g1::RosG1TwistController>(
        stateSubscriber, twistModelPath, twistWalk7Motion, twistWalk7Start, twistWalk7End, "G1TwistWalk7"));

    // Swing motion
    auto twistSwingMotion = tbai::fromGlobalConfig<std::string>("g1_twist_swing/motion_file");
    auto twistSwingStart = tbai::fromGlobalConfig<float>("g1_twist_swing/time_start");
    auto twistSwingEnd = tbai::fromGlobalConfig<float>("g1_twist_swing/time_end");
    TBAI_LOG_INFO(logger, "Loading TWIST2 Swing: {}", twistSwingMotion);
    controller.addController(std::make_unique<tbai::g1::RosG1TwistController>(
        stateSubscriber, twistModelPath, twistSwingMotion, twistSwingStart, twistSwingEnd, "G1TwistSwing"));

    // Load G1 PBHC Horse Stance Punch controller
    auto pbhcModelPath = tbai::fromGlobalConfig<std::string>("g1_pbhc_horse_stance_punch/model_path");
    auto pbhcMotionFile = tbai::fromGlobalConfig<std::string>("g1_pbhc_horse_stance_punch/motion_file");
    auto pbhcTimeStart = tbai::fromGlobalConfig<float>("g1_pbhc_horse_stance_punch/time_start");
    auto pbhcTimeEnd = tbai::fromGlobalConfig<float>("g1_pbhc_horse_stance_punch/time_end");
    TBAI_LOG_INFO(logger, "Loading PBHC Horse Stance Punch: {}", pbhcMotionFile);
    controller.addController(std::make_unique<tbai::g1::RosG1PBHCController>(
        stateSubscriber, pbhcModelPath, pbhcMotionFile, pbhcTimeStart, pbhcTimeEnd, "G1PBHCHorseStancePunch"));

    // Load G1 PBHC Horse Stance Pose controller (model_50000)
    auto pbhcPoseModelPath = tbai::fromGlobalConfig<std::string>("g1_pbhc_horse_stance_pose/model_path");
    auto pbhcPoseMotionFile = tbai::fromGlobalConfig<std::string>("g1_pbhc_horse_stance_pose/motion_file");
    auto pbhcPoseTimeStart = tbai::fromGlobalConfig<float>("g1_pbhc_horse_stance_pose/time_start");
    auto pbhcPoseTimeEnd = tbai::fromGlobalConfig<float>("g1_pbhc_horse_stance_pose/time_end");
    TBAI_LOG_INFO(logger, "Loading PBHC Horse Stance Pose: {}", pbhcPoseMotionFile);
    controller.addController(std::make_unique<tbai::g1::RosG1PBHCController>(
        stateSubscriber, pbhcPoseModelPath, pbhcPoseMotionFile, pbhcPoseTimeStart, pbhcPoseTimeEnd, "G1PBHCHorseStancePose"));

    // Load G1 PBHC Horse Stance Pose v2 controller (model_119000)
    auto pbhcPose2ModelPath = tbai::fromGlobalConfig<std::string>("g1_pbhc_horse_stance_pose2/model_path");
    auto pbhcPose2MotionFile = tbai::fromGlobalConfig<std::string>("g1_pbhc_horse_stance_pose2/motion_file");
    auto pbhcPose2TimeStart = tbai::fromGlobalConfig<float>("g1_pbhc_horse_stance_pose2/time_start");
    auto pbhcPose2TimeEnd = tbai::fromGlobalConfig<float>("g1_pbhc_horse_stance_pose2/time_end");
    TBAI_LOG_INFO(logger, "Loading PBHC Horse Stance Pose v2: {}", pbhcPose2MotionFile);
    controller.addController(std::make_unique<tbai::g1::RosG1PBHCController>(
        stateSubscriber, pbhcPose2ModelPath, pbhcPose2MotionFile, pbhcPose2TimeStart, pbhcPose2TimeEnd, "G1PBHCHorseStancePose2"));

    // Load G1 ASAP Locomotion controller (decoupled locomotion with stand height)
    auto asapModelPath = tbai::fromGlobalConfig<std::string>("g1_asap_locomotion/model_path");
    TBAI_LOG_INFO(logger, "Loading ASAP Locomotion: {}", asapModelPath);
    controller.addController(std::make_unique<tbai::g1::RosG1ASAPController>(
        stateSubscriber, referenceVelocityPtr, asapModelPath, "G1ASAPLocomotion"));

    TBAI_LOG_INFO(logger, "Controllers initialized. Starting main loop...");

    // Start controller loop
    controller.start();

    return EXIT_SUCCESS;
}
