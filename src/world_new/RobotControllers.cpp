//
// Created by john on 1/17/20.
//

#include "include/roboteam_ai/world_new/RobotControllers.hpp"

namespace rtt::world_new::robot {
std::unique_ptr<ai::control::ShotController> &RobotControllers::getShotController() noexcept { return shotController; }

std::unique_ptr<ai::control::NumTreePosControl> &RobotControllers::getNumTreePosController() noexcept { return numTreePosControl; }

std::unique_ptr<ai::control::BasicPosControl> &RobotControllers::getBasicPosController() noexcept { return basicPosControl; }

std::unique_ptr<ai::control::BallHandlePosControl> &RobotControllers::getBallHandlePosController() noexcept { return ballHandlePosControl; }
}  // namespace rtt::world_new::robot