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
#include <tbai_ros_g1/G1ASAPMimicController.hpp>
#include <tbai_ros_g1/G1BeyondMimicController.hpp>
#include <tbai_ros_g1/G1MimicController.hpp>
#include <tbai_ros_g1/G1PBHCController.hpp>
#include <tbai_ros_g1/G1RLController.hpp>
#include <tbai_ros_g1/G1SpinkickController.hpp>
#include <tbai_ros_g1/G1Twist2Controller.hpp>
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

    // Walk motion 1
    auto twistHfRepo = tbai::fromGlobalConfig<std::string>("g1_twist_walk1/hf_repo");
    auto twistHfModel = tbai::fromGlobalConfig<std::string>("g1_twist_walk1/hf_model");
    auto twistHfMotion1 = tbai::fromGlobalConfig<std::string>("g1_twist_walk1/hf_motion_file");
    auto twistModelPath = tbai::downloadFromHuggingFace(twistHfRepo, twistHfModel);
    auto twistWalk1Motion = tbai::downloadFromHuggingFace(twistHfRepo, twistHfMotion1);
    auto twistWalk1Start = tbai::fromGlobalConfig<float>("g1_twist_walk1/time_start");
    auto twistWalk1End = tbai::fromGlobalConfig<float>("g1_twist_walk1/time_end");
    TBAI_LOG_INFO(logger, "Loading TWIST2 Walk1: {}/{}", twistHfRepo, twistHfMotion1);
    controller.addController(std::make_unique<tbai::g1::RosG1Twist2Controller>(
        stateSubscriber, twistModelPath, twistWalk1Motion, twistWalk1Start, twistWalk1End, "G1TwistWalk1"));

    // Walk motion 2
    auto twistHfMotion2 = tbai::fromGlobalConfig<std::string>("g1_twist_walk2/hf_motion_file");
    auto twistWalk2Motion = tbai::downloadFromHuggingFace(twistHfRepo, twistHfMotion2);
    auto twistWalk2Start = tbai::fromGlobalConfig<float>("g1_twist_walk2/time_start");
    auto twistWalk2End = tbai::fromGlobalConfig<float>("g1_twist_walk2/time_end");
    TBAI_LOG_INFO(logger, "Loading TWIST2 Walk2: {}/{}", twistHfRepo, twistHfMotion2);
    controller.addController(std::make_unique<tbai::g1::RosG1Twist2Controller>(
        stateSubscriber, twistModelPath, twistWalk2Motion, twistWalk2Start, twistWalk2End, "G1TwistWalk2"));

    // Walk motion 3
    auto twistHfMotion3 = tbai::fromGlobalConfig<std::string>("g1_twist_walk3/hf_motion_file");
    auto twistWalk3Motion = tbai::downloadFromHuggingFace(twistHfRepo, twistHfMotion3);
    auto twistWalk3Start = tbai::fromGlobalConfig<float>("g1_twist_walk3/time_start");
    auto twistWalk3End = tbai::fromGlobalConfig<float>("g1_twist_walk3/time_end");
    TBAI_LOG_INFO(logger, "Loading TWIST2 Walk3: {}/{}", twistHfRepo, twistHfMotion3);
    controller.addController(std::make_unique<tbai::g1::RosG1Twist2Controller>(
        stateSubscriber, twistModelPath, twistWalk3Motion, twistWalk3Start, twistWalk3End, "G1TwistWalk3"));

    // Walk motion 5
    auto twistHfMotion5 = tbai::fromGlobalConfig<std::string>("g1_twist_walk5/hf_motion_file");
    auto twistWalk5Motion = tbai::downloadFromHuggingFace(twistHfRepo, twistHfMotion5);
    auto twistWalk5Start = tbai::fromGlobalConfig<float>("g1_twist_walk5/time_start");
    auto twistWalk5End = tbai::fromGlobalConfig<float>("g1_twist_walk5/time_end");
    TBAI_LOG_INFO(logger, "Loading TWIST2 Walk5: {}/{}", twistHfRepo, twistHfMotion5);
    controller.addController(std::make_unique<tbai::g1::RosG1Twist2Controller>(
        stateSubscriber, twistModelPath, twistWalk5Motion, twistWalk5Start, twistWalk5End, "G1TwistWalk5"));

    // Walk motion 7
    auto twistHfMotion7 = tbai::fromGlobalConfig<std::string>("g1_twist_walk7/hf_motion_file");
    auto twistWalk7Motion = tbai::downloadFromHuggingFace(twistHfRepo, twistHfMotion7);
    auto twistWalk7Start = tbai::fromGlobalConfig<float>("g1_twist_walk7/time_start");
    auto twistWalk7End = tbai::fromGlobalConfig<float>("g1_twist_walk7/time_end");
    TBAI_LOG_INFO(logger, "Loading TWIST2 Walk7: {}/{}", twistHfRepo, twistHfMotion7);
    controller.addController(std::make_unique<tbai::g1::RosG1Twist2Controller>(
        stateSubscriber, twistModelPath, twistWalk7Motion, twistWalk7Start, twistWalk7End, "G1TwistWalk7"));

    // Swing motion
    auto twistHfMotionSwing = tbai::fromGlobalConfig<std::string>("g1_twist_swing/hf_motion_file");
    auto twistSwingMotion = tbai::downloadFromHuggingFace(twistHfRepo, twistHfMotionSwing);
    auto twistSwingStart = tbai::fromGlobalConfig<float>("g1_twist_swing/time_start");
    auto twistSwingEnd = tbai::fromGlobalConfig<float>("g1_twist_swing/time_end");
    TBAI_LOG_INFO(logger, "Loading TWIST2 Swing: {}/{}", twistHfRepo, twistHfMotionSwing);
    controller.addController(std::make_unique<tbai::g1::RosG1Twist2Controller>(
        stateSubscriber, twistModelPath, twistSwingMotion, twistSwingStart, twistSwingEnd, "G1TwistSwing"));

    // Load G1 PBHC Horse Stance Punch controller
    auto pbhcHfRepo = tbai::fromGlobalConfig<std::string>("g1_pbhc_horse_stance_punch/hf_repo");
    auto pbhcHfModel = tbai::fromGlobalConfig<std::string>("g1_pbhc_horse_stance_punch/hf_model");
    auto pbhcHfMotion = tbai::fromGlobalConfig<std::string>("g1_pbhc_horse_stance_punch/hf_motion_file");
    auto pbhcModelPath = tbai::downloadFromHuggingFace(pbhcHfRepo, pbhcHfModel);
    auto pbhcMotionFile = tbai::downloadFromHuggingFace(pbhcHfRepo, pbhcHfMotion);
    auto pbhcTimeStart = tbai::fromGlobalConfig<float>("g1_pbhc_horse_stance_punch/time_start");
    auto pbhcTimeEnd = tbai::fromGlobalConfig<float>("g1_pbhc_horse_stance_punch/time_end");
    TBAI_LOG_INFO(logger, "Loading PBHC Horse Stance Punch: {}/{}", pbhcHfRepo, pbhcHfModel);
    controller.addController(std::make_unique<tbai::g1::RosG1PBHCController>(
        stateSubscriber, pbhcModelPath, pbhcMotionFile, pbhcTimeStart, pbhcTimeEnd, "G1PBHCHorseStancePunch"));

    // Load G1 PBHC Horse Stance Pose controller (model_50000)
    auto pbhcPoseHfRepo = tbai::fromGlobalConfig<std::string>("g1_pbhc_horse_stance_pose/hf_repo");
    auto pbhcPoseHfModel = tbai::fromGlobalConfig<std::string>("g1_pbhc_horse_stance_pose/hf_model");
    auto pbhcPoseHfMotion = tbai::fromGlobalConfig<std::string>("g1_pbhc_horse_stance_pose/hf_motion_file");
    auto pbhcPoseModelPath = tbai::downloadFromHuggingFace(pbhcPoseHfRepo, pbhcPoseHfModel);
    auto pbhcPoseMotionFile = tbai::downloadFromHuggingFace(pbhcPoseHfRepo, pbhcPoseHfMotion);
    auto pbhcPoseTimeStart = tbai::fromGlobalConfig<float>("g1_pbhc_horse_stance_pose/time_start");
    auto pbhcPoseTimeEnd = tbai::fromGlobalConfig<float>("g1_pbhc_horse_stance_pose/time_end");
    TBAI_LOG_INFO(logger, "Loading PBHC Horse Stance Pose: {}/{}", pbhcPoseHfRepo, pbhcPoseHfModel);
    controller.addController(std::make_unique<tbai::g1::RosG1PBHCController>(
        stateSubscriber, pbhcPoseModelPath, pbhcPoseMotionFile, pbhcPoseTimeStart, pbhcPoseTimeEnd, "G1PBHCHorseStancePose"));

    // Load G1 PBHC Horse Stance Pose v2 controller (model_119000)
    auto pbhcPose2HfRepo = tbai::fromGlobalConfig<std::string>("g1_pbhc_horse_stance_pose2/hf_repo");
    auto pbhcPose2HfModel = tbai::fromGlobalConfig<std::string>("g1_pbhc_horse_stance_pose2/hf_model");
    auto pbhcPose2HfMotion = tbai::fromGlobalConfig<std::string>("g1_pbhc_horse_stance_pose2/hf_motion_file");
    auto pbhcPose2ModelPath = tbai::downloadFromHuggingFace(pbhcPose2HfRepo, pbhcPose2HfModel);
    auto pbhcPose2MotionFile = tbai::downloadFromHuggingFace(pbhcPose2HfRepo, pbhcPose2HfMotion);
    auto pbhcPose2TimeStart = tbai::fromGlobalConfig<float>("g1_pbhc_horse_stance_pose2/time_start");
    auto pbhcPose2TimeEnd = tbai::fromGlobalConfig<float>("g1_pbhc_horse_stance_pose2/time_end");
    TBAI_LOG_INFO(logger, "Loading PBHC Horse Stance Pose v2: {}/{}", pbhcPose2HfRepo, pbhcPose2HfModel);
    controller.addController(std::make_unique<tbai::g1::RosG1PBHCController>(
        stateSubscriber, pbhcPose2ModelPath, pbhcPose2MotionFile, pbhcPose2TimeStart, pbhcPose2TimeEnd, "G1PBHCHorseStancePose2"));

    // Load G1 ASAP Locomotion controller (decoupled locomotion with stand height)
    auto asapHfRepo = tbai::fromGlobalConfig<std::string>("g1_asap_locomotion/hf_repo");
    auto asapHfModel = tbai::fromGlobalConfig<std::string>("g1_asap_locomotion/hf_model");
    auto asapModelPath = tbai::downloadFromHuggingFace(asapHfRepo, asapHfModel);
    TBAI_LOG_INFO(logger, "Loading ASAP Locomotion: {}/{}", asapHfRepo, asapHfModel);
    controller.addController(std::make_unique<tbai::g1::RosG1ASAPController>(
        stateSubscriber, referenceVelocityPtr, asapModelPath, "G1ASAPLocomotion"));

    // Load G1 ASAP Mimic controllers
    // CR7
    auto cr7HfRepo = tbai::fromGlobalConfig<std::string>("g1_asap_cr7/hf_repo");
    auto cr7HfModel = tbai::fromGlobalConfig<std::string>("g1_asap_cr7/hf_model");
    auto cr7ModelPath = tbai::downloadFromHuggingFace(cr7HfRepo, cr7HfModel);
    auto cr7MotionLength = tbai::fromGlobalConfig<float>("g1_asap_cr7/motion_length");
    TBAI_LOG_INFO(logger, "Loading ASAP CR7: {}/{}", cr7HfRepo, cr7HfModel);
    controller.addController(std::make_unique<tbai::g1::RosG1ASAPMimicController>(
        stateSubscriber, cr7ModelPath, cr7MotionLength, "G1ASAPCR7"));

    // APT
    auto aptHfRepo = tbai::fromGlobalConfig<std::string>("g1_asap_apt/hf_repo");
    auto aptHfModel = tbai::fromGlobalConfig<std::string>("g1_asap_apt/hf_model");
    auto aptModelPath = tbai::downloadFromHuggingFace(aptHfRepo, aptHfModel);
    auto aptMotionLength = tbai::fromGlobalConfig<float>("g1_asap_apt/motion_length");
    TBAI_LOG_INFO(logger, "Loading ASAP APT: {}/{}", aptHfRepo, aptHfModel);
    controller.addController(std::make_unique<tbai::g1::RosG1ASAPMimicController>(
        stateSubscriber, aptModelPath, aptMotionLength, "G1ASAPAPT"));

    // Jump Forward Level 1
    auto jf1HfRepo = tbai::fromGlobalConfig<std::string>("g1_asap_jump_forward1/hf_repo");
    auto jf1HfModel = tbai::fromGlobalConfig<std::string>("g1_asap_jump_forward1/hf_model");
    auto jf1ModelPath = tbai::downloadFromHuggingFace(jf1HfRepo, jf1HfModel);
    auto jf1MotionLength = tbai::fromGlobalConfig<float>("g1_asap_jump_forward1/motion_length");
    TBAI_LOG_INFO(logger, "Loading ASAP Jump Forward 1: {}/{}", jf1HfRepo, jf1HfModel);
    controller.addController(std::make_unique<tbai::g1::RosG1ASAPMimicController>(
        stateSubscriber, jf1ModelPath, jf1MotionLength, "G1ASAPJumpForward1"));

    // Jump Forward Level 2
    auto jf2HfRepo = tbai::fromGlobalConfig<std::string>("g1_asap_jump_forward2/hf_repo");
    auto jf2HfModel = tbai::fromGlobalConfig<std::string>("g1_asap_jump_forward2/hf_model");
    auto jf2ModelPath = tbai::downloadFromHuggingFace(jf2HfRepo, jf2HfModel);
    auto jf2MotionLength = tbai::fromGlobalConfig<float>("g1_asap_jump_forward2/motion_length");
    TBAI_LOG_INFO(logger, "Loading ASAP Jump Forward 2: {}/{}", jf2HfRepo, jf2HfModel);
    controller.addController(std::make_unique<tbai::g1::RosG1ASAPMimicController>(
        stateSubscriber, jf2ModelPath, jf2MotionLength, "G1ASAPJumpForward2"));

    // Jump Forward Level 3
    auto jf3HfRepo = tbai::fromGlobalConfig<std::string>("g1_asap_jump_forward3/hf_repo");
    auto jf3HfModel = tbai::fromGlobalConfig<std::string>("g1_asap_jump_forward3/hf_model");
    auto jf3ModelPath = tbai::downloadFromHuggingFace(jf3HfRepo, jf3HfModel);
    auto jf3MotionLength = tbai::fromGlobalConfig<float>("g1_asap_jump_forward3/motion_length");
    TBAI_LOG_INFO(logger, "Loading ASAP Jump Forward 3: {}/{}", jf3HfRepo, jf3HfModel);
    controller.addController(std::make_unique<tbai::g1::RosG1ASAPMimicController>(
        stateSubscriber, jf3ModelPath, jf3MotionLength, "G1ASAPJumpForward3"));

    // Kick Level 1
    auto kick1HfRepo = tbai::fromGlobalConfig<std::string>("g1_asap_kick1/hf_repo");
    auto kick1HfModel = tbai::fromGlobalConfig<std::string>("g1_asap_kick1/hf_model");
    auto kick1ModelPath = tbai::downloadFromHuggingFace(kick1HfRepo, kick1HfModel);
    auto kick1MotionLength = tbai::fromGlobalConfig<float>("g1_asap_kick1/motion_length");
    TBAI_LOG_INFO(logger, "Loading ASAP Kick 1: {}/{}", kick1HfRepo, kick1HfModel);
    controller.addController(std::make_unique<tbai::g1::RosG1ASAPMimicController>(
        stateSubscriber, kick1ModelPath, kick1MotionLength, "G1ASAPKick1"));

    // Kick Level 2
    auto kick2HfRepo = tbai::fromGlobalConfig<std::string>("g1_asap_kick2/hf_repo");
    auto kick2HfModel = tbai::fromGlobalConfig<std::string>("g1_asap_kick2/hf_model");
    auto kick2ModelPath = tbai::downloadFromHuggingFace(kick2HfRepo, kick2HfModel);
    auto kick2MotionLength = tbai::fromGlobalConfig<float>("g1_asap_kick2/motion_length");
    TBAI_LOG_INFO(logger, "Loading ASAP Kick 2: {}/{}", kick2HfRepo, kick2HfModel);
    controller.addController(std::make_unique<tbai::g1::RosG1ASAPMimicController>(
        stateSubscriber, kick2ModelPath, kick2MotionLength, "G1ASAPKick2"));

    // Kick Level 3
    auto kick3HfRepo = tbai::fromGlobalConfig<std::string>("g1_asap_kick3/hf_repo");
    auto kick3HfModel = tbai::fromGlobalConfig<std::string>("g1_asap_kick3/hf_model");
    auto kick3ModelPath = tbai::downloadFromHuggingFace(kick3HfRepo, kick3HfModel);
    auto kick3MotionLength = tbai::fromGlobalConfig<float>("g1_asap_kick3/motion_length");
    TBAI_LOG_INFO(logger, "Loading ASAP Kick 3: {}/{}", kick3HfRepo, kick3HfModel);
    controller.addController(std::make_unique<tbai::g1::RosG1ASAPMimicController>(
        stateSubscriber, kick3ModelPath, kick3MotionLength, "G1ASAPKick3"));

    // Kobe
    auto kobeHfRepo = tbai::fromGlobalConfig<std::string>("g1_asap_kobe/hf_repo");
    auto kobeHfModel = tbai::fromGlobalConfig<std::string>("g1_asap_kobe/hf_model");
    auto kobeModelPath = tbai::downloadFromHuggingFace(kobeHfRepo, kobeHfModel);
    auto kobeMotionLength = tbai::fromGlobalConfig<float>("g1_asap_kobe/motion_length");
    TBAI_LOG_INFO(logger, "Loading ASAP Kobe: {}/{}", kobeHfRepo, kobeHfModel);
    controller.addController(std::make_unique<tbai::g1::RosG1ASAPMimicController>(
        stateSubscriber, kobeModelPath, kobeMotionLength, "G1ASAPKobe"));

    // LeBron Level 1
    auto lebron1HfRepo = tbai::fromGlobalConfig<std::string>("g1_asap_lebron1/hf_repo");
    auto lebron1HfModel = tbai::fromGlobalConfig<std::string>("g1_asap_lebron1/hf_model");
    auto lebron1ModelPath = tbai::downloadFromHuggingFace(lebron1HfRepo, lebron1HfModel);
    auto lebron1MotionLength = tbai::fromGlobalConfig<float>("g1_asap_lebron1/motion_length");
    TBAI_LOG_INFO(logger, "Loading ASAP LeBron 1: {}/{}", lebron1HfRepo, lebron1HfModel);
    controller.addController(std::make_unique<tbai::g1::RosG1ASAPMimicController>(
        stateSubscriber, lebron1ModelPath, lebron1MotionLength, "G1ASAPLeBron1"));

    // LeBron Level 2
    auto lebron2HfRepo = tbai::fromGlobalConfig<std::string>("g1_asap_lebron2/hf_repo");
    auto lebron2HfModel = tbai::fromGlobalConfig<std::string>("g1_asap_lebron2/hf_model");
    auto lebron2ModelPath = tbai::downloadFromHuggingFace(lebron2HfRepo, lebron2HfModel);
    auto lebron2MotionLength = tbai::fromGlobalConfig<float>("g1_asap_lebron2/motion_length");
    TBAI_LOG_INFO(logger, "Loading ASAP LeBron 2: {}/{}", lebron2HfRepo, lebron2HfModel);
    controller.addController(std::make_unique<tbai::g1::RosG1ASAPMimicController>(
        stateSubscriber, lebron2ModelPath, lebron2MotionLength, "G1ASAPLeBron2"));

    // Side Jump Level 1
    auto sj1HfRepo = tbai::fromGlobalConfig<std::string>("g1_asap_side_jump1/hf_repo");
    auto sj1HfModel = tbai::fromGlobalConfig<std::string>("g1_asap_side_jump1/hf_model");
    auto sj1ModelPath = tbai::downloadFromHuggingFace(sj1HfRepo, sj1HfModel);
    auto sj1MotionLength = tbai::fromGlobalConfig<float>("g1_asap_side_jump1/motion_length");
    TBAI_LOG_INFO(logger, "Loading ASAP Side Jump 1: {}/{}", sj1HfRepo, sj1HfModel);
    controller.addController(std::make_unique<tbai::g1::RosG1ASAPMimicController>(
        stateSubscriber, sj1ModelPath, sj1MotionLength, "G1ASAPSideJump1"));

    // Side Jump Level 2
    auto sj2HfRepo = tbai::fromGlobalConfig<std::string>("g1_asap_side_jump2/hf_repo");
    auto sj2HfModel = tbai::fromGlobalConfig<std::string>("g1_asap_side_jump2/hf_model");
    auto sj2ModelPath = tbai::downloadFromHuggingFace(sj2HfRepo, sj2HfModel);
    auto sj2MotionLength = tbai::fromGlobalConfig<float>("g1_asap_side_jump2/motion_length");
    TBAI_LOG_INFO(logger, "Loading ASAP Side Jump 2: {}/{}", sj2HfRepo, sj2HfModel);
    controller.addController(std::make_unique<tbai::g1::RosG1ASAPMimicController>(
        stateSubscriber, sj2ModelPath, sj2MotionLength, "G1ASAPSideJump2"));

    // Side Jump Level 3
    auto sj3HfRepo = tbai::fromGlobalConfig<std::string>("g1_asap_side_jump3/hf_repo");
    auto sj3HfModel = tbai::fromGlobalConfig<std::string>("g1_asap_side_jump3/hf_model");
    auto sj3ModelPath = tbai::downloadFromHuggingFace(sj3HfRepo, sj3HfModel);
    auto sj3MotionLength = tbai::fromGlobalConfig<float>("g1_asap_side_jump3/motion_length");
    TBAI_LOG_INFO(logger, "Loading ASAP Side Jump 3: {}/{}", sj3HfRepo, sj3HfModel);
    controller.addController(std::make_unique<tbai::g1::RosG1ASAPMimicController>(
        stateSubscriber, sj3ModelPath, sj3MotionLength, "G1ASAPSideJump3"));

    TBAI_LOG_INFO(logger, "Controllers initialized. Starting main loop...");

    // Start controller loop
    controller.start();

    return EXIT_SUCCESS;
}
