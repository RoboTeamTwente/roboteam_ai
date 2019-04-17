//
// Created by thijs on 17-12-18.
//

#include <roboteam_ai/src/coach/GeneralPositionCoach.h>
#include "Attack.h"
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include <roboteam_ai/src/control/ControlUtils.h>

namespace rtt {
namespace ai {

Attack::Attack(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

void Attack::onInitialize() {
    gtp = std::make_shared<control::NumTreePosControl>();
    gtp->setAvoidBall(Constants::DEFAULT_BALLCOLLISION_RADIUS());
    shot = false;
}

/// Get an update on the skill
bt::Node::Status Attack::onUpdate() {
    if (! robot) return Status::Running;

    if (shot && !world::world->robotHasBall(robot->id, world::OUR_ROBOTS)) {
        return Status::Success;
    }

    Vector2 ball = world::world->getBall()->pos;
    Vector2 behindBall = coach::g_generalPositionCoach.getPositionBehindBallToGoal(BEHIND_BALL_TARGET, false);
    Vector2 deltaBall = behindBall - ball;

    if (!coach::g_generalPositionCoach.isRobotBehindBallToGoal(BEHIND_BALL_CHECK, false, robot->pos)) {
        targetPos = behindBall;
        command.w = static_cast<float>((ball - (Vector2) (robot->pos)).angle());
        gtp->setAvoidBall(Constants::DEFAULT_BALLCOLLISION_RADIUS());

        if (abs(((Vector2) robot->pos - targetPos).length()) < SWITCH_TO_BASICGTP_DISTANCE) {
            gtp->setAvoidBall(false);
            gtp = std::make_shared<control::BasicPosControl>();
        }
    }
    else {
        targetPos = ball;
        gtp->setAvoidBall(false);
        command.w = (world::field->get_their_goal_center() - ball).toAngle().getAngle();
        //command.w = static_cast<float>(((Vector2) {- 1.0, - 1.0}*deltaBall).angle());
        if (world::world->robotHasBall(robot->id, true, Constants::MAX_KICK_RANGE())) {
            command.kicker = 1;
            command.kicker_vel = static_cast<float>(rtt::ai::Constants::MAX_KICK_POWER());
            command.kicker_forced = 1;
            shot = true;
        }

    }
    Vector2 velocity;
    if (world::field->pointIsInDefenseArea(robot->pos, false, 0.0)) {
        velocity = ((Vector2) robot->pos - world::field->get_our_goal_center()).stretchToLength(2.0);
    }
    else if (world::field->pointIsInDefenseArea(robot->pos, false, 0.0)) {
        velocity = ((Vector2) robot->pos - world::field->get_their_goal_center()).stretchToLength(2.0);
    }
    else if (world::field->pointIsInDefenseArea(ball, false) || world::field->pointIsInDefenseArea(ball, true)) {
        velocity = {0, 0};
    }
    else if (world::field->pointIsInDefenseArea(targetPos, false)) {
        velocity = {0, 0};
    }
    else {
        velocity = gtp->getPosVelAngle(robot, targetPos).vel;
    }

    velocity = control::ControlUtils::velocityLimiter(velocity);

    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);

    publishRobotCommand();

    return Status::Running;
}

void Attack::onTerminate(Status s) {

    command.w = static_cast<float>(deltaPos.angle());
    command.x_vel = 0;
    command.y_vel = 0;

    publishRobotCommand();
}

} // ai
} // rtt