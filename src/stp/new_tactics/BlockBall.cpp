//
// Created by timovdk on 5/12/20.
//

#include "stp/new_tactics/BlockBall.h"

#include "control/ControlUtils.h"
#include "stp/new_skills/GoToPos.h"
#include "stp/new_skills/Rotate.h"

namespace rtt::ai::stp::tactic {

BlockBall::BlockBall() { skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()}; }

void BlockBall::onInitialize() noexcept {}

void BlockBall::onUpdate(Status const &status) noexcept {}

void BlockBall::onTerminate() noexcept {
    // Call terminate on all skills
    for (auto &x : skills) {
        x->terminate();
    }
}

StpInfo BlockBall::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    auto field = info.getField().value();
    auto ball = info.getBall().value();
    auto robot = info.getRobot().value();
    auto enemyRobot = info.getEnemyRobot().value();

    auto robotToBall = ball->getPos() - robot->getPos();

    auto targetPosition = calculateTargetPosition(ball, field, enemyRobot, robot);

    auto targetAngle = robotToBall.angle();

    skillStpInfo.setPositionToMoveTo(targetPosition);
    skillStpInfo.setAngle(targetAngle);

    return skillStpInfo;
}

bool BlockBall::isEndTactic() noexcept { return true; }

bool BlockBall::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool BlockBall::shouldTacticReset(const StpInfo &info) noexcept {
    double errorMargin = control_constants::GO_TO_POS_ERROR_MARGIN;
    return (info.getRobot().value()->getPos() - info.getPositionToMoveTo().value()).length() > errorMargin;
}

const char *BlockBall::getName() { return "Block Ball"; }

Vector2 BlockBall::calculateTargetPosition(const world_new::view::BallView &ball, const world::Field &field, const world_new::view::RobotView &enemyRobot,
                                           const world_new::view::RobotView &robot) noexcept {
    Vector2 targetPosition{};

    // Opponent is close to ball
    // Block the ball by staying on the shot line of the opponent
    if (enemyRobot->getDistanceToBall() < control_constants::ENEMY_CLOSE_TO_BALL_DISTANCE) {
        /**
         * If an opponent is close, weigh the angle of that enemy robot as well (to prevent shot/pass)
         */
        // targetPosition = ;
    }

    // Ball is moving
    // Intercept ball when it is moving towards the goal
    else if (ball->getVelocity().length() > control_constants::BALL_STILL_VEL) {
        /**
         * If ball is moving, weigh direction and speed (and maybe increase distance from ball / switch to intercept mode)
         */
        // targetPosition = ;
    }

    // Default
    // Stay between the ball and the center of the goal
    else {
        targetPosition = LineSegment(ball->getPos(), field.getOurGoalCenter()).project(robot->getPos());
    }

    return targetPosition;
}

}  // namespace rtt::ai::stp::tactic