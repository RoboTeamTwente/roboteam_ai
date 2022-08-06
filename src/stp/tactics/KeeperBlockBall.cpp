#include "stp/tactics/KeeperBlockBall.h"

#include "control/ControlUtils.h"
#include "roboteam_utils/LineSegment.h"
#include "stp/constants/ControlConstants.h"
#include "stp/skills/GoToPos.h"
#include "utilities/Constants.h"

#include <roboteam_utils/HalfLine.h>
#include <roboteam_utils/Mathematics.h>

namespace rtt::ai::stp::tactic {

// We do not want the keeper to stand completely inside the goal, but a tiny bit outside.
const double KEEPER_DISTANCE_TO_GOAL_LINE = Constants::ROBOT_RADIUS() * std::sin(toRadians(80.0));
// And by standing a tiny bit inside, we cannot move completely to a goal side. This is by how much less that is.
const double KEEPER_GOAL_DECREASE_AT_ONE_SIDE = Constants::ROBOT_RADIUS() * std::cos(toRadians(80.0)) + 0.01; // Plus a small margin to prevent keeper from crashing into goal
// The maximum distance from the goal for when we say the ball is heading towards our goal
constexpr double MAX_DISTANCE_HEADING_TOWARDS_GOAL = 2;
// For determining where the keeper should stand to stand between the ball and the goal, we draw a line from the ball to a bit behind the goal
constexpr double PROJECT_BALL_DISTANCE_TO_GOAL = 0.5; // Small means keeper will me more in center, big means keeper will be more to the side of the goal
// We stop deciding where the keeper should be if the ball is too far behind our own goal
constexpr double MAX_DISTANCE_BALL_BEHIND_GOAL = 0.3;

KeeperBlockBall::KeeperBlockBall() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()}; }

std::optional<StpInfo> KeeperBlockBall::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getField() || !skillStpInfo.getBall() || !skillStpInfo.getRobot()) return std::nullopt;

    skillStpInfo.setShouldAvoidOutOfField(false);

    auto targetPosition = calculateTargetPosition(info.getBall().value(), info.getField().value(), info.getEnemyRobot());
    skillStpInfo.setPositionToMoveTo(targetPosition.first);
    skillStpInfo.setPidType(targetPosition.second);

    auto targetAngle = calculateTargetAngle(info.getBall().value(), targetPosition.first);
    skillStpInfo.setAngle(targetAngle);

    return skillStpInfo;
}

bool KeeperBlockBall::isEndTactic() noexcept { return true; }

bool KeeperBlockBall::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool KeeperBlockBall::shouldTacticReset(const StpInfo &info) noexcept {
    double errorMargin = control_constants::GO_TO_POS_ERROR_MARGIN;
    return (info.getRobot().value()->getPos() - info.getPositionToMoveTo().value()).length() > errorMargin;
}

const char *KeeperBlockBall::getName() { return "Keeper Block Ball"; }

LineSegment KeeperBlockBall::getKeepersLineSegment(const world::Field &field) {
    auto keepersLineSegmentLeft = field.getOurTopGoalSide() + Vector2(KEEPER_DISTANCE_TO_GOAL_LINE, -KEEPER_GOAL_DECREASE_AT_ONE_SIDE);
    auto keepersLineSegmentRight = field.getOurBottomGoalSide() + Vector2(KEEPER_DISTANCE_TO_GOAL_LINE, KEEPER_GOAL_DECREASE_AT_ONE_SIDE);
    auto keepersLineSegment = LineSegment(keepersLineSegmentLeft, keepersLineSegmentRight);
    return keepersLineSegment;
}

std::optional<HalfLine> KeeperBlockBall::estimateBallTrajectory(const world::view::BallView &ball, const std::optional<world::view::RobotView> &enemyRobot) {
    // If the ball is already moving, the trajectory of the ball is clear
    if (ball->velocity.length2() > control_constants::BALL_STILL_VEL2) {
        auto start = ball->position;
        auto direction = ball->position + ball->velocity;
        return HalfLine(start, direction);
    }

    bool hasEnemy = enemyRobot.has_value() && enemyRobot.value().get() != nullptr;

    // If the enemy robot already has the ball, it will probably kick in the direction it is facing
    if (hasEnemy && enemyRobot->get()->hasBall()) {
        auto start = enemyRobot->get()->getPos();
        auto direction = enemyRobot->get()->getPos() + enemyRobot->get()->getAngle().toVector2();
        return HalfLine(start, direction);
    }

    // If the enemy is only a bit close, we look at the direction from the robot to the ball
    if (enemyRobot && enemyRobot->get()->getDistanceToBall() < control_constants::ENEMY_CLOSE_TO_BALL_DISTANCE) {
        auto start = enemyRobot->get()->getPos();
        auto direction = ball->position;
        return HalfLine(start, direction);
    }

    // Otherwise, the ball probably will not be moved by enemies soon :P
    return std::nullopt;
}

bool KeeperBlockBall::isBallHeadingTowardsOurGoal(const HalfLine &ballTrajectory, const world::Field &field) {
    auto goalLineSegment = LineSegment(field.getOurBottomGoalSide(), field.getOurTopGoalSide());
    // Get the intersection between the ball and the line on which the goal is
    auto intersectionWithGoalLine = ballTrajectory.intersect(Line(goalLineSegment));
    // Ball heads towards goal if the intersection is near our goal
    return intersectionWithGoalLine.has_value() && goalLineSegment.distanceToLine(intersectionWithGoalLine.value()) < MAX_DISTANCE_HEADING_TOWARDS_GOAL;
}

std::pair<Vector2,PIDType> KeeperBlockBall::calculateTargetPosition(const world::view::BallView &ball, const world::Field &field,
                                                                          const std::optional<world::view::RobotView> &enemyRobot) noexcept {
    // Get the line on which the keeper should move to
    auto keepersLineSegment = getKeepersLineSegment(field);

    // Find out what the trajectory of the ball will be
    auto ballTrajectory = estimateBallTrajectory(ball, enemyRobot);

    // If the ball will go towards our goal, try and block it
    bool ballHeadsTowardsOurGoal = ballTrajectory.has_value() && isBallHeadingTowardsOurGoal(ballTrajectory.value(), field);
    if (ballHeadsTowardsOurGoal) {
        // Get the keeper as close as possible to the trajectory of the ball
        auto targetPosition = keepersLineSegment.getClosestPointToLine(ballTrajectory->toLine());
        if (targetPosition.has_value()){
            if (ball->velocity.length() > control_constants::BALL_IS_MOVING_SLOW_LIMIT){
                return { targetPosition.value(), PIDType::KEEPER };
            }
            return { targetPosition.value(), PIDType::DEFAULT};
        }
    }

    // Otherwise, the ball probably will not move towards the goal any time soon
    if (ball->position.x >= field.getOurGoalCenter().x - MAX_DISTANCE_BALL_BEHIND_GOAL) {
        // This will pick a position somewhat between the ball and the goal,
        // but only if the ball is not too far behind the goal
        auto ballGoalLine = Line(ball->position, field.getOurGoalCenter() - Vector2(PROJECT_BALL_DISTANCE_TO_GOAL, 0));
        auto targetPosition = keepersLineSegment.getClosestPointToLine(ballGoalLine);
        if (targetPosition.has_value()) return { targetPosition.value(), PIDType::DEFAULT };
    }

    // If that fails too, just go to the center of the goal
    return { Vector2(keepersLineSegment.start.x, 0), PIDType::DEFAULT };
}

Angle KeeperBlockBall::calculateTargetAngle(const world::view::BallView &ball, const Vector2 &targetKeeperPosition) {
    // Look towards ball to ensure ball hits the front assembly to reduce odds of ball reflecting in goal
    auto keeperToBall = ball->position - targetKeeperPosition;
    return keeperToBall.angle();
}

}  // namespace rtt::ai::stp::tactic
