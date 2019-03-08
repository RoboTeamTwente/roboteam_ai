//
// Created by thijs on 17-12-18.
//

#include "Attack.h"

namespace rtt {
namespace ai {

Attack::Attack(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

void Attack::onInitialize() {
    ownGoal = properties->getBool("ownGoal");
    goToPos.setAvoidBall(true);
    shot = false;

    roboteam_msgs::GeometryFieldSize field = Field::get_field();
    smallerGenevaMargin = (field.field_length / 2) - (1.5 * field.goal_width);
}

/// Get an update on the skill

bt::Node::Status Attack::onUpdate() {
    if (! robot) return Status::Running;

    if (shot && !World::botHasBall(robot->id, true)) {
        return Status::Success;
    }


    ballTarget = Field::get_their_goal_center();
    genevaState = 3;

    if (properties->hasInt("genevaState")) {
        genevaState = properties->getInt("genevaState");
    }

    /// Overwrite set genevaState if autoGeneva is true
    /// Robot is set to shoot in the "short corner" but look at the other corner

    if (properties->getBool("autoGeneva")) {
        roboteam_msgs::GeometryFieldSize field = Field::get_field();
        double xTarget = ownGoal ? -field.field_length / 2 : field.field_length / 2;
        if (ball->pos.y > 0) {
            genevaState = ownGoal ? 1 : 5;
            ballTarget = {xTarget, 0.30 * field.goal_width};
        } else {
            genevaState = ownGoal ? 5 : 1;
            ballTarget = {xTarget, -0.30 * field.goal_width};
        }

        /// Set the geneva to one angle lower if the distance from the goal is more than 1,5 times the goal width
        if ((!ownGoal && ball->pos.x < smallerGenevaMargin) || (ownGoal && ball->pos.x >  -smallerGenevaMargin)) {
            /// Set geneva to 4 if it is 5, set geneva to 2 if it is 1
            if (genevaState == 5) {
                genevaState = 4;
            } else if (genevaState == 1) {
                genevaState = 2;
            }
        }
    }

    Vector2 ball = World::getBall()->pos;
    Vector2 aimPos = control::ControlUtils::getGenevaAim(ball, ballTarget, genevaState);
    Vector2 behindBall = Coach::getPositionBehindBallToPosition(0.2, aimPos);
    Vector2 deltaBall = behindBall - ball;

    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.geneva_state = genevaState;

    if (! Coach::isRobotBehindBallToPosition(0.3, aimPos, robot->pos)) {
        targetPos = behindBall;
        command.use_angle = 1;
        command.w = static_cast<float>((ball - (Vector2) (robot->pos)).angle());
        goToPos.setAvoidBall(true);

        if (abs(((Vector2) robot->pos - targetPos).length()) < 0.10) {
            goToPos.setAvoidBall(false);
        }
    }
    else {
        targetPos = ball;
        goToPos.setAvoidBall(false);
        command.use_angle = 1;
        command.w = static_cast<float>(((Vector2) {- 1.0, - 1.0}*deltaBall).angle());
        if (World::botHasBall(robot->id, true)) {
            command.kicker = 1;
            command.kicker_vel = static_cast<float>(rtt::ai::Constants::MAX_KICK_POWER());
            command.kicker_forced = 1;
            shot = true;
        }

    }

    Vector2 velocity;
    if (Field::pointIsInDefenceArea(robot->pos, true, 0.0)) {
        velocity = ((Vector2) robot->pos - targetPos).stretchToLength(2.0);
    }
    else if (Field::pointIsInDefenceArea(robot->pos, false, 0.0)) {
        velocity = ((Vector2) robot->pos - targetPos).stretchToLength(2.0);
    }
    else if (Field::pointIsInDefenceArea(ball, ownGoal) || Field::pointIsInDefenceArea(ball, !ownGoal)) {
        velocity = {0, 0};
    }
    else if (Field::pointIsInDefenceArea(targetPos, ownGoal)) {
        velocity = {0, 0};
    }
    else {
        velocity = goToPos.goToPos(robot, targetPos, GoToType::BASIC).vel;
    }

    velocity = control::ControlUtils::VelocityLimiter(velocity);

    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    publishRobotCommand(command);

    return Status::Running;
}

void Attack::onTerminate(Status s) {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.w = static_cast<float>(deltaPos.angle());
    command.x_vel = 0;
    command.y_vel = 0;

    publishRobotCommand(command);
}

} // ai
} // rtt