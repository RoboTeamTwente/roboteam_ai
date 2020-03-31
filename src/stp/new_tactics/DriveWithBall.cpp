//
// Created by timovdk on 3/16/20.
//

#include <roboteam_utils/Print.h>
#include <stp/new_skills/GoToPos.h>
#include <stp/new_skills/Rotate.h>
#include <stp/new_tactics/DriveWithBall.h>

namespace rtt::ai::stp::tactic {

DriveWithBall::DriveWithBall() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::Rotate(), skill::GoToPos()};
    skills.initialize();
}

void DriveWithBall::onInitialize() noexcept {}

void DriveWithBall::onUpdate(Status const& status) noexcept {}

void DriveWithBall::onTerminate() noexcept {
    // Call terminate on all skills
    for (auto& x : skills) {
        x->terminate();
    }
}

StpInfo DriveWithBall::calculateInfoForSkill(StpInfo const& info) noexcept {
    if (!info.getBall() || !info.getRobot() || !info.getField()) {
        RTT_WARNING("No Ball, Robot or Field present in StpInfo");
        return {};
    }

    StpInfo skillStpInfo = info;

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

    return fabs(robotAngle + ballToRobotAngle) <= stp::control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN;
}

bool DriveWithBall::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

const char *DriveWithBall::getName() {
    return "Drive With Ball";
}

}  // namespace rtt::ai::stp::tactic
