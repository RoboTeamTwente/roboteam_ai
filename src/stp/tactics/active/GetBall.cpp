//
// Created by ratoone on 10-03-20.
/// Moves the robot to the BALL in a straight line and ROTATES the robot towards to ball.

/// ACTIVE
//

#include "stp/tactics/active/GetBall.h"

#include "control/ControlUtils.h"
#include "stp/skills/GoToPos.h"
#include "stp/skills/Rotate.h"
#include "world/FieldComputations.h"

namespace rtt::ai::stp::tactic {
GetBall::GetBall() { skills = collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()}; }

std::optional<StpInfo> GetBall::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getRobot() || !skillStpInfo.getBall()) return std::nullopt;

    Vector2 robotPosition = skillStpInfo.getRobot().value()->getPos();
    Vector2 ballPosition = skillStpInfo.getBall().value()->getPos();
    double ballDistance = (ballPosition - robotPosition).length();

    if (skillStpInfo.getRobot()->get()->getAngleDiffToBall() > control_constants::HAS_BALL_ANGLE_ERROR_MARGIN * M_PI && ballDistance < control_constants::AVOID_BALL_DISTANCE) {
        // don't move too close to the ball until the angle to the ball is (roughly) correct
        auto targetPos = control::ControlUtils::projectPointToValidPosition(info.getField().value(), info.getRobot()->get()->getPos(), info.getRoleName(),
                                                                            control_constants::DEFENSE_AREA_AVOIDANCE_MARGIN);
        skillStpInfo.setPositionToMoveTo(targetPos);
    } else {
        // the robot will go to the position of the ball
        Vector2 newRobotPosition = robotPosition + (ballPosition - robotPosition).stretchToLength(ballDistance - control_constants::CENTER_TO_FRONT + 0.035);

        // Don't get the ball at an invalid position
        if (!FieldComputations::pointIsValidPosition(info.getField().value(), newRobotPosition, info.getRoleName(), control_constants::DEFENSE_AREA_AVOIDANCE_MARGIN)) {
            newRobotPosition =
                control::ControlUtils::projectPointToValidPosition(info.getField().value(), newRobotPosition, info.getRoleName(), control_constants::DEFENSE_AREA_AVOIDANCE_MARGIN);
        }

        skillStpInfo.setPositionToMoveTo(newRobotPosition);
    }

    skillStpInfo.setAngle((ballPosition - robotPosition).angle());

    if (ballDistance < control_constants::TURN_ON_DRIBBLER_DISTANCE) {
        skillStpInfo.setDribblerSpeed(100);
    }

    return skillStpInfo;
}

bool GetBall::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool GetBall::shouldTacticReset(const StpInfo &info) noexcept {
    return (info.getRobot()->get()->getAngleDiffToBall() < control_constants::HAS_BALL_ANGLE_ERROR_MARGIN * M_PI) && (skills.current_num() == 1);
}

bool GetBall::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

bool GetBall::forceTacticSuccess(const StpInfo &info) noexcept { return info.getRobot()->hasBall(); }

const char *GetBall::getName() { return "Get Ball"; }

}  // namespace rtt::ai::stp::tactic
