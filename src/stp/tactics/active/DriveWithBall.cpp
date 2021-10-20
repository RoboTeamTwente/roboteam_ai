//
// Created by timovdk on 3/16/20.
/// Rotates the target position and drives towards it
/// TODO-Max Add functionallity of backwards dribbling
/// TODO-Max Dynamic dribblerspeed

/// ACTIVE
//

#include "stp/tactics/active/DriveWithBall.h"

#include "stp/skills/GoToPos.h"
#include "stp/skills/Rotate.h"

namespace rtt::ai::stp::tactic {

DriveWithBall::DriveWithBall() {
    // Create state machine of skills and initialize first skill
    // TODO: The rotate skill might be unnecessary in this tactic if our dribbler works well whilst driving backwards
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{/*skill::Rotate(), */skill::GoToPos()};
}

std::optional<StpInfo> DriveWithBall::calculateInfoForSkill(StpInfo const& info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getPositionToMoveTo() || !skillStpInfo.getBall()) return std::nullopt;

    double angleToBall = (info.getPositionToMoveTo().value() - info.getBall()->get()->getPos()).angle();
    skillStpInfo.setAngle(angleToBall);

    // When driving with ball, we need to activate the dribbler
    // For now, this means full power, but this might change later
    // TODO: TUNE better way to determine dribblerspeed
    skillStpInfo.setDribblerSpeed(100);

    return skillStpInfo;
}

bool DriveWithBall::isTacticFailing(const StpInfo& info) noexcept {
    // Fail if we don't have the ball or there is no movement position
    return !info.getRobot()->hasBall() || !info.getPositionToMoveTo();
}

bool DriveWithBall::shouldTacticReset(const StpInfo& info) noexcept {
    // Should reset if the angle the robot is at is no longer correct
    auto robotAngle = info.getRobot()->get()->getAngle();
    auto ballToRobotAngle = (info.getBall()->get()->getPos() - info.getRobot()->get()->getPos()).angle();
    return fabs(robotAngle + Angle(ballToRobotAngle)) <= stp::control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN;
}

bool DriveWithBall::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

const char* DriveWithBall::getName() { return "Drive With Ball"; }

}  // namespace rtt::ai::stp::tactic
