//
// Created by jordi on 06-04-20.
//

#include <roboteam_utils/LineSegment.h>
#include "stp/tactics/GetBehindBallInDirection.h"
#include "stp/skills/GoToPos.h"
#include "stp/skills/Rotate.h"

namespace rtt::ai::stp::tactic {
GetBehindBallInDirection::GetBehindBallInDirection() { skills = collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()}; }

std::optional<StpInfo> GetBehindBallInDirection::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;
    double DISTANCE_TO_ROBOT_NEAR = 2.6 * control_constants::ROBOT_RADIUS;

    // Don't calculate if some info is missing.
    if (!skillStpInfo.getRobot() || !skillStpInfo.getBall()) return std::nullopt;

    Vector2 robotPosition = info.getRobot().value()->getPos();
    Vector2 ballPosition = info.getBall().value()->getPos();
    Vector2 targetPosition = info.getField()->getTheirGoalCenter();
    Vector2 newRobotPosition;

    if(skillStpInfo.getPositionToShootAt().has_value()) {
        targetPosition = info.getPositionToShootAt().value();
    }

    // If ball is to far away, dont get stuck on other GoToPos(), but restart and get behind the ball
    if (skillStpInfo.getBall().value()->getPos().dist2(skillStpInfo.getRobot().value()->getPos()) > control_constants::DISTANCE_TO_ROBOT_FAR){
        skills.skip_to(0);
    }

    if (skills.current_num() == 0) {
        newRobotPosition = ballPosition + (ballPosition - targetPosition).stretchToLength(control_constants::TURN_ON_DRIBBLER_DISTANCE);

        // Check if the path crosses through the ball
        auto pathToBehindBall = LineSegment(robotPosition, newRobotPosition);
        auto distFromPathToBall = pathToBehindBall.distanceToLine(ballPosition);
        auto drivingDirection = robotPosition - newRobotPosition;
        if(distFromPathToBall < control_constants::ROBOT_RADIUS * 1.3) {
            // Decide which side of the ball to approach on
            auto posBallLeft = ballPosition + Vector2(drivingDirection.y, -drivingDirection.x).stretchToLength(DISTANCE_TO_ROBOT_NEAR);
            auto posBallRight = ballPosition + Vector2(-drivingDirection.y, drivingDirection.x).stretchToLength(DISTANCE_TO_ROBOT_NEAR);
            newRobotPosition = (posBallLeft.dist(robotPosition) < posBallRight.dist(robotPosition)) ? posBallLeft : posBallRight;
        }
        skillStpInfo.setPositionToMoveTo(newRobotPosition);
    } else {
        // Rotate towards ball
        skillStpInfo.setAngle((newRobotPosition - ballPosition).angle());
    }

    return skillStpInfo;
}

bool GetBehindBallInDirection::isTacticFailing(const StpInfo &info) noexcept { return !info.getRobot() || !info.getBall(); }

bool GetBehindBallInDirection::shouldTacticReset(const StpInfo &info) noexcept { return false; }

bool GetBehindBallInDirection::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

const char *GetBehindBallInDirection::getName() { return "Get Ball In Direction"; }

}  // namespace rtt::ai::stp::tactic
