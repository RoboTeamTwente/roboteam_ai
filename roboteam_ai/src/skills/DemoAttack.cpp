//
// Created by robzelluf on 3/21/19.
//

#include <roboteam_ai/src/control/PositionUtils.h>
#include "DemoAttack.h"
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include <roboteam_ai/src/control/ControlUtils.h>

namespace rtt {
namespace ai {

DemoAttack::DemoAttack(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

void DemoAttack::onInitialize() {
    robot->getNumtreeGtp()->setAvoidBall(Constants::DEFAULT_BALLCOLLISION_RADIUS());
    ownGoal = properties->getBool("ownGoal");
    shot = false;
}

/// Get an update on the skill
bt::Node::Status DemoAttack::onUpdate() {
    if (! robot) return Status::Running;

    if (shot && !world::world->robotHasBall(robot->id, true)) {
        return Status::Success;
    }

    Vector2 ball = world::world->getBall()->pos;
    Vector2 behindBall = control::PositionUtils::getPositionBehindBallToGoal(BEHIND_BALL_TARGET, ownGoal);
    Vector2 deltaBall = behindBall - ball;

    if (!control::PositionUtils::isRobotBehindBallToGoal(BEHIND_BALL_CHECK, ownGoal, robot->pos)) {
        targetPos = behindBall;
        command.w = static_cast<float>((ball - (Vector2) (robot->pos)).angle());
        robot->getNumtreeGtp()->setAvoidBall(Constants::DEFAULT_BALLCOLLISION_RADIUS());

        if (abs(((Vector2) robot->pos - targetPos).length()) < SWITCH_TO_BASICGTP_DISTANCE) {
            robot->getNumtreeGtp()->setAvoidBall(0);
        }
    }
    else {
        targetPos = ball;
        robot->getNumtreeGtp()->setAvoidBall(0);
        command.w = static_cast<float>(((Vector2) {- 1.0, - 1.0}*deltaBall).angle());
        if (world::world->robotHasBall(robot->id, true)) {
            command.kicker = 1;
            command.kicker_vel = static_cast<float>(rtt::ai::Constants::MAX_KICK_POWER());
            command.kicker_forced = 1;
            shot = true;
        }

    }
    Vector2 velocity;
    if (world::field->pointIsInDefenceArea(robot->pos, ownGoal, 0.0)) {
        velocity = ((Vector2) robot->pos - world::field->get_our_goal_center()).stretchToLength(2.0);
    }
    else if (world::field->pointIsInDefenceArea(robot->pos, ownGoal, 0.0)) {
        velocity = ((Vector2) robot->pos - world::field->get_their_goal_center()).stretchToLength(2.0);
    }
    else if (world::field->pointIsInDefenceArea(ball, ownGoal) || world::field->pointIsInDefenceArea(ball, !ownGoal)) {
        velocity = {0, 0};
    }
    else if (world::field->pointIsInDefenceArea(targetPos, ownGoal)) {
        velocity = {0, 0};
    }
    else {
        velocity = robot->getNumtreeGtp()->getPosVelAngle(robot, targetPos).vel;
    }

    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    publishRobotCommand();

    return Status::Running;
}

void DemoAttack::onTerminate(Status s) {
    command.w = static_cast<float>(deltaPos.angle());
    command.x_vel = 0;
    command.y_vel = 0;
    publishRobotCommand();
}

} // ai
} // rtt
