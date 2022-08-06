//
// Created by jordi on 06-04-20.
// Rebuilt by alexander on 21-12-21
//
#include "stp/tactics/active/GetBehindBallInDirection.h"

#include <roboteam_utils/LineSegment.h>

#include "roboteam_utils/Circle.h"
#include "stp/constants/ControlConstants.h"
#include "stp/skills/GoToPos.h"
#include "stp/skills/Orbit.h"

namespace rtt::ai::stp::tactic {

GetBehindBallInDirection::GetBehindBallInDirection() { skills = collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Orbit()}; }

std::optional<StpInfo> GetBehindBallInDirection::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getRobot() || !skillStpInfo.getBall() || !skillStpInfo.getPositionToShootAt()) {
        return std::nullopt;
    }

    // Look at the position to shoot at
    skillStpInfo.setAngle((info.getPositionToShootAt().value() - info.getRobot()->get()->getPos()).angle());

    // If the robot is far from the ball, go to the target position
    if ((info.getBall()->get()->position - info.getRobot()->get()->getPos()).length() > 0.5) {
        auto targetPos = GetBehindBallInDirection::calculateTargetPosition(info.getBall()->get()->position, info.getRobot()->get()->getPos(), info.getPositionToShootAt().value());
        skillStpInfo.setPositionToMoveTo(targetPos);
    }

    // If the robot is close to the ball, use orbit to get behind it
    else {
        if (skills.current_num() == 0) {
            skills.skip_to(1);
        }
    }

    return skillStpInfo;
}

Vector2 GetBehindBallInDirection::calculateTargetPosition(Vector2 ballPosition, Vector2 robotPosition, Vector2 positionToShootAt) {
    auto ballAvoidDistance = 4 * control_constants::ROBOT_RADIUS;
    auto ballToTarget = positionToShootAt - ballPosition;

    // The inital target position is behind the ball in the direction to shoot at, at a distance of ballAvoidDistance
    auto targetPos = ballPosition - ballToTarget.stretchToLength(ballAvoidDistance);

    // If the line from the robot to the target brings the robot closer to the ball than the avoid distance, adjust the target position to prevent this
    auto robotToTarget = targetPos - robotPosition;
    if (!Circle(ballPosition, ballAvoidDistance).intersects(LineSegment(robotPosition, targetPos)).empty()) {
        auto direction = ballToTarget.toAngle().rotateDirection(robotToTarget) ? 1.0 : -1.0;
        auto finalPos = ballPosition + robotToTarget.rotate(M_PI_2).stretchToLength(ballAvoidDistance) * direction;
        return finalPos;
    } else {
        return targetPos;
    }
}
bool GetBehindBallInDirection::isTacticFailing(const StpInfo &info) noexcept { return !info.getPositionToShootAt(); }

bool GetBehindBallInDirection::shouldTacticReset(const StpInfo &info) noexcept {
    return ((info.getBall()->get()->position - info.getRobot()->get()->getPos()).length() > 0.5) && (skills.current_num() == 1);
}

bool GetBehindBallInDirection::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

const char *GetBehindBallInDirection::getName() { return "Get Ball In Direction"; }
}  // namespace rtt::ai::stp::tactic