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
#include <tbai_bob/BobController.hpp>
#include <tbai_ros_bob/Visualizers.hpp>
#include <tbai_ros_core/Subscribers.hpp>
#include <tbai_ros_gridmap/GridmapInterface.hpp>
#include <tbai_ros_reference/ReferenceVelocityGenerator.hpp>
#include <torch/script.h>

namespace tbai {
namespace rl {

using namespace tbai;             // NOLINT
using namespace torch::indexing;  // NOLINT
using torch::jit::script::Module;

class RosBobController : public tbai::BobController {
   public:
    RosBobController(const std::string &urdfString, const std::shared_ptr<tbai::StateSubscriber> &stateSubscriberPtr,
                     const std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> &refVelGenPtr);

    void postStep(scalar_t currentTime, scalar_t dt) override;
    void changeController(const std::string &controllerType, scalar_t currentTime) override;
    void atPositions(matrix_t &positions) override;
    void stopController() override;
    bool ok() const override;
    void preStep(scalar_t currentTime, scalar_t dt) override;

   private:
    std::unique_ptr<tbai::gridmap::GridmapInterface> gridmap_;
    StateVisualizer stateVisualizer_;
    HeightsReconstructedVisualizer heightsReconstructedVisualizer_;
    ContactVisualizer contactVisualizer_;

    scalar_t timeSinceLastVisualizationUpdate_;

    bool publishState_;
    ros::Publisher statePublisher_;

    void publishEstimatedState();
};

}  // namespace rl
}  // namespace tbai
