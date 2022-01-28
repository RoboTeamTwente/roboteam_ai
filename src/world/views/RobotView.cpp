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

// TODO: Move this functionality to roboteam_world. That repo should decide whether a robot has the ball
// TODO: TEST whether maxDist and Angle are suitable on the field and whether ballsensor works well irl
bool RobotView::hasBall(double maxDist, double maxAngle) const noexcept {
    // If we use a simulator and this robot is ours, we only use the ballsensor to determine if a robot has the ball
    // In all other cases, we check for distance and ballsensor, one of which needs to be true
    bool robotHasBall = false;

    if (this->robotPtr->getTeam() == Team::us && SETTINGS.getRobotHubMode() == Settings::RobotHubMode::SIMULATOR) {
        robotHasBall = get()->ballSensorSeesBall();
    } else {
        robotHasBall = get()->ballSensorSeesBall() || hasBallAccordingToVision(maxDist, maxAngle);
    }

    return robotHasBall;
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
