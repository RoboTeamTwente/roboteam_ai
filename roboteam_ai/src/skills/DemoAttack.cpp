//
// Created by robzelluf on 3/21/19.
//

#include <roboteam_ai/src/coach/GeneralPositionCoach.h>
#include "DemoAttack.h"
#include <roboteam_ai/src/utilities/Field.h>
#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include <roboteam_ai/src/control/ControlUtils.h>

namespace rtt {
namespace ai {

DemoAttack::DemoAttack(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

void DemoAttack::onInitialize() {
    ownGoal = properties->getBool("ownGoal");
    goToPos = std::make_shared<control::NumTreePosControl>();
    goToPos->setAvoidBall(true);
    shot = false;
}

/// Get an update on the skill
bt::Node::Status DemoAttack::onUpdate() {
    if (! robot) return Status::Running;

    if (shot && !World::botHasBall(robot->id, true)) {
        return Status::Success;
    }

    Vector2 ball = World::getBall()->pos;
    Vector2 behindBall = coach::g_generalPositionCoach.getPositionBehindBallToGoal(BEHIND_BALL_TARGET, ownGoal);
    Vector2 deltaBall = behindBall - ball;

    if (!coach::g_generalPositionCoach.isRobotBehindBallToGoal(BEHIND_BALL_CHECK, ownGoal, robot->pos)) {
        targetPos = behindBall;
        command.w = static_cast<float>((ball - (Vector2) (robot->pos)).angle());
        goToPos->setAvoidBall(true);

        if (abs(((Vector2) robot->pos - targetPos).length()) < SWITCH_TO_BASICGTP_DISTANCE) {
            goToPos = std::make_shared<control::BasicPosControl>();
            goToPos->setAvoidBall(false);
        }
    }
    else {
        targetPos = ball;
        goToPos->setAvoidBall(false);
        command.w = static_cast<float>(((Vector2) {- 1.0, - 1.0}*deltaBall).angle());
        if (World::botHasBall(robot->id, true)) {
            command.kicker = 1;
            command.kicker_vel = static_cast<float>(rtt::ai::Constants::MAX_KICK_POWER());
            command.kicker_forced = 1;
            shot = true;
        }

    }
    Vector2 velocity;
    if (Field::pointIsInDefenceArea(robot->pos, ownGoal, 0.0)) {
        velocity = ((Vector2) robot->pos - Field::get_our_goal_center()).stretchToLength(2.0);
    }
    else if (Field::pointIsInDefenceArea(robot->pos, ownGoal, 0.0)) {
        velocity = ((Vector2) robot->pos - Field::get_their_goal_center()).stretchToLength(2.0);
    }
    else if (Field::pointIsInDefenceArea(ball, ownGoal) || Field::pointIsInDefenceArea(ball, !ownGoal)) {
        velocity = {0, 0};
    }
    else if (Field::pointIsInDefenceArea(targetPos, ownGoal)) {
        velocity = {0, 0};
    }
    else {
        velocity = goToPos->getPosVelAngle(robot, targetPos).vel;
    }

    velocity = control::ControlUtils::velocityLimiter(velocity);

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
