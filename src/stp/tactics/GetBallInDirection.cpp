//
// Created by jordi on 06-04-20.
/// TODO-Max simplify to get behind as T:GetBall goes towards ball.
//

#include "stp/tactics/GetBallInDirection.h"

#include "stp/skills/GoToPos.h"
#include "stp/skills/Rotate.h"

namespace rtt::ai::stp::tactic {
GetBallInDirection::GetBallInDirection() { skills = collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate(), skill::GoToPos()}; }

std::optional<StpInfo> GetBallInDirection::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getRobot() || !skillStpInfo.getBall() || !skillStpInfo.getPositionToShootAt()) return std::nullopt;

    Vector2 robotPosition = info.getRobot().value()->getPos();
    Vector2 ballPosition = info.getBall().value()->getPos();
    Vector2 targetPosition = info.getPositionToShootAt().value();

    // The robot will go to the position of the ball
    double ballDistance = (ballPosition - robotPosition).length();
    Vector2 newRobotPosition;

    if (skills.current_num() == 0) {
        // First GoToPos: Go behind ball
        newRobotPosition = ballPosition + (ballPosition - targetPosition).stretchToLength(control_constants::TURN_ON_DRIBBLER_DISTANCE);
    } else {
        // Second GoToPos: Go towards ball
        newRobotPosition = robotPosition + (ballPosition - robotPosition).stretchToLength(ballDistance);

        // Rotate towards ball
        skillStpInfo.setAngle((ballPosition - robotPosition).angle());
    }

    // Turn on dribbler when close to ball
    if (ballDistance < control_constants::TURN_ON_DRIBBLER_DISTANCE) {
        skillStpInfo.setDribblerSpeed(100);
    }

    skillStpInfo.setPositionToMoveTo(newRobotPosition);

    return skillStpInfo;
}

bool GetBallInDirection::isTacticFailing(const StpInfo &info) noexcept { return !info.getPositionToShootAt(); }

bool GetBallInDirection::shouldTacticReset(const StpInfo &info) noexcept { return false; }

bool GetBallInDirection::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

const char *GetBallInDirection::getName() { return "Get Ball In Direction"; }

}  // namespace rtt::ai::stp::tactic
