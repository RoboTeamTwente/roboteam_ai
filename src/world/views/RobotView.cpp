//
// Created by john on 1/14/20.
//

#include "world/views/RobotView.hpp"

#include "world/World.hpp"

namespace rtt::world::view {
RobotView::RobotView(const rtt::world::robot::Robot *const _ptr) noexcept : robotPtr{_ptr} {}

robot::Robot const *RobotView::get() const noexcept { return robotPtr; }

robot::Robot const &RobotView::operator*() const noexcept { return *get(); }

robot::Robot const *RobotView::operator->() const noexcept { return get(); }

// TODO: TEST whether maxDist and Angle are suitable on the field and whether ballsensor works well irl
bool RobotView::hasBall(double maxDist, double maxAngle) const noexcept {
    // In the simulator, return true if the ballSensorSeesBall (feedback from sim)
    // On the field, return true if the ballsensor sees the ball or we have the ball according to the vision
    return (SETTINGS.isSerialMode() && hasBallAccordingToVision(maxDist, maxAngle)) || get()->ballSensorSeesBall();
}

Vector2 RobotView::getKicker() const noexcept {
    Vector2 distanceToKicker{ai::stp::control_constants::CENTER_TO_FRONT + 0.1, 0};
    return get()->getPos() + distanceToKicker.rotate(get()->getAngle());
}

RobotView::operator bool() const noexcept { return get() != nullptr; }

bool RobotView::hasBallAccordingToVision(double maxDist, double maxAngle) const noexcept {
    auto dist = get()->getDistanceToBall();
    auto angle = get()->getAngleDiffToBall();
    return dist < maxDist && angle < maxAngle * M_PI;
}

}  // namespace rtt::world::view