#include <iostream>
#include <memory>

#include "tbai_ros_np3o/Np3oController.hpp"
#include <ros/ros.h>
#include <tbai_core/Logging.hpp>
#include <tbai_core/ResourceMonitor.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/CentralController.hpp>
#include <tbai_ros_core/Publishers.hpp>
#include <tbai_ros_core/Rate.hpp>
#include <tbai_ros_core/Subscribers.hpp>
#include <tbai_ros_reference/ReferenceVelocityGenerator.hpp>
#include <tbai_ros_static/StaticController.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "tbai_ros_np3o");
    ros::NodeHandle nh;

    // Set zero time
    tbai::writeInitTime(tbai::RosTime::rightNow());

    auto stateTopic = tbai::fromGlobalConfig<std::string>("state_topic");
    auto commandTopic = tbai::fromGlobalConfig<std::string>("command_topic");
    auto changeControllerTopic = tbai::fromGlobalConfig<std::string>("change_controller_topic");
    const std::string urdfString = nh.param<std::string>("robot_description", "");

    auto estimatorType = tbai::fromGlobalConfig<std::string>("estimator_type");

    std::shared_ptr<tbai::StateSubscriber> stateSubscriber;
    if (estimatorType == "muse") {
        stateSubscriber =
            std::shared_ptr<tbai::StateSubscriber>(new tbai::InekfRosStateSubscriber(nh, stateTopic, urdfString));
    } else if (estimatorType == "ground_truth") {
        stateSubscriber = std::shared_ptr<tbai::StateSubscriber>(new tbai::RosStateSubscriber(nh, stateTopic));
    } else {
        TBAI_GLOBAL_LOG_FATAL("Invalid estimator type: {}. Must be one of 'muse' or 'ground_truth'", estimatorType);
        return EXIT_FAILURE;
    }

    std::shared_ptr<tbai::CommandPublisher> commandPublisher =
        std::shared_ptr<tbai::CommandPublisher>(new tbai::RosCommandPublisher(nh, commandTopic));

    std::shared_ptr<tbai::ChangeControllerSubscriber> changeControllerSubscriber =
        std::shared_ptr<tbai::ChangeControllerSubscriber>(
            new tbai::RosChangeControllerSubscriber(nh, changeControllerTopic));

    tbai::CentralController<ros::Rate, tbai::RosTime> controller(commandPublisher, changeControllerSubscriber);

    // Add static controller
    controller.addController(std::make_unique<tbai::static_::RosStaticController>(stateSubscriber));

    // Add Np3o controller
    controller.addController(std::make_unique<tbai::np3o::RosNp3oController>(
        urdfString, stateSubscriber, tbai::reference::getReferenceVelocityGeneratorShared(nh)));

    // Start the resource monitor
    tbai::ResourceMonitor resourceMonitor(1.0 / 30.0, 1.0 / 10.0);
    resourceMonitor.startThread();

    // Start controller loop
    controller.start();

    // Wait for resource monitor to finish
    resourceMonitor.stopThread();

    return EXIT_SUCCESS;
}
