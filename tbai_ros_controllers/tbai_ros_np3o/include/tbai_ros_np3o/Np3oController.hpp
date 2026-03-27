#pragma once

// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <tbai_np3o/Np3oController.hpp>
#include <tbai_ros_core/Subscribers.hpp>
#include <tbai_ros_gridmap/GridmapInterface.hpp>
#include <tbai_ros_np3o/Visualizers.hpp>
#include <tbai_ros_reference/ReferenceVelocityGenerator.hpp>
#include <torch/script.h>

namespace tbai {
namespace np3o {

using namespace tbai;             // NOLINT
using namespace torch::indexing;  // NOLINT
using torch::jit::script::Module;

class RosNp3oController : public tbai::Np3oController {
   public:
    RosNp3oController(const std::string &urdfString, const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                      const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr);

    void postStep(scalar_t currentTime, scalar_t dt) override;
    void changeController(const std::string &controllerType, scalar_t currentTime) override;
    void stopController() override;
    bool ok() const override;
    void preStep(scalar_t currentTime, scalar_t dt) override;

   private:
    StateVisualizer stateVisualizer_;
    ContactVisualizer contactVisualizer_;
    ros::Publisher statePublisher_;
    bool publishState_;

    void publishEstimatedState();

    scalar_t timeSinceLastVisualizationUpdate_;
};

}  // namespace np3o
}  // namespace tbai
