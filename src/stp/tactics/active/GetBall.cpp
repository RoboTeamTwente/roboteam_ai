//
// Created by ratoone on 10-03-20.
/// Moves the robot to the BALL in a straight line and ROTATES the robot towards to ball.

/// ACTIVE
//

#include "stp/tactics/active/GetBall.h"

#include "control/ControlUtils.h"
#include "stp/skills/GoToPos.h"
#include "stp/skills/Rotate.h"
#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::tactic {
GetBall::GetBall() { skills = collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()}; }

std::optional<StpInfo> GetBall::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getRobot() || !skillStpInfo.getBall()) return std::nullopt;

    Vector2 robotPosition = info.getRobot().value()->getPos();
    Vector2 ballPosition = info.getBall().value()->getPos();

    // If this robot is not the keeper, don't get the ball inside a defense area
    if (info.getRobot()->get()->getId() != GameStateManager::getCurrentGameState().keeperId && FieldComputations::pointIsInDefenseArea(info.getField().value(), ballPosition)) {
        ballPosition = control::ControlUtils::projectPositionToOutsideDefenseArea(info.getField().value(), ballPosition, control_constants::AVOID_BALL_DISTANCE);
    }

    // the robot will go to the position of the ball
    double ballDistance = (ballPosition - robotPosition).length();
    Vector2 newRobotPosition = robotPosition + (ballPosition - robotPosition).stretchToLength(ballDistance - control_constants::CENTER_TO_FRONT + 0.035);

    if (ballDistance < control_constants::TURN_ON_DRIBBLER_DISTANCE) {
        skillStpInfo.setAngle((ballPosition - robotPosition).angle());
        skillStpInfo.setDribblerSpeed(100);
    }

    skillStpInfo.setPositionToMoveTo(newRobotPosition);

    return skillStpInfo;
}

bool GetBall::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool GetBall::shouldTacticReset(const StpInfo &info) noexcept { return !info.getRobot()->hasBall(); }

bool GetBall::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

bool GetBall::forceTacticSuccess(const StpInfo &info) noexcept { return info.getRobot()->hasBall(); }

const char *GetBall::getName() { return "Get Ball"; }

}  // namespace rtt::ai::stp::tactic
