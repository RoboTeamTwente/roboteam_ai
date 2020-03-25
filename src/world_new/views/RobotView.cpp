//
// Created by john on 1/14/20.
//

#include "world_new/views/RobotView.hpp"

#include <include/roboteam_ai/utilities/Constants.h>

#include <include/roboteam_ai/world_new/Robot.hpp>
#include <include/roboteam_ai/world_new/World.hpp>

namespace rtt::world_new::view {
RobotView::RobotView(const rtt::world_new::robot::Robot *const _ptr) noexcept : robotPtr{_ptr} {}

robot::Robot const *RobotView::get() const noexcept { return robotPtr; }

robot::Robot const &RobotView::operator*() const noexcept { return *get(); }

robot::Robot const *RobotView::operator->() const noexcept { return get(); }

// TODO: TEST to see if we don't have issues using this naive approach irl
bool RobotView::hasBall(double maxDist, bool noBallSensor) const noexcept {
    // Take ballSensor input when noBallSensor is false (default) and the ballSensor works
    // else take radius check
    if (!noBallSensor && get()->isWorkingBallSensor()) return get()->ballSensorSeesBall();
    return get()->getDistanceToBall() < maxDist;
}

Vector2 RobotView::getKicker() const noexcept {
    Vector2 distanceToKicker{ai::Constants::CENTRE_TO_FRONT() + 0.1, 0};
    return get()->getPos() + distanceToKicker.rotate(get()->getAngle());
}

RobotView::operator bool() const noexcept { return get() != nullptr; }

robot::RobotControllers &RobotView::getControllers() const noexcept { return World::instance()->getControllersForRobot(get()->getId()); }

}  // namespace rtt::world_new::view
