//
// Created by baris on 24/10/18.
//

#include "GoToPos.h"
#include "roboteam_ai/src/world/Field.h"
#include "../control/ControlUtils.h"

namespace rtt {
namespace ai {

GoToPos::GoToPos(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

/// Init the GoToPos skill
void GoToPos::onInitialize() {
    goToBall = properties->getBool("goToBall");
    goBehindBall = properties->getBool("goBehindBall");

    if (properties->hasVector2("distanceBehindBall")) {
        distanceBehindBall = properties->getDouble("distanceBehindBall");
    } else {
        distanceBehindBall = 0.2;
    }

    if (properties->hasVector2("Position")) {
        targetPos = properties->getVector2("Position");
    }

    if (properties->hasDouble("maxVel"))
        speed=properties->getDouble("maxVel");
    else
        speed=Constants::DEFAULT_MAX_VEL();
}

/// Get an update on the skill
bt::Node::Status GoToPos::onUpdate() {
    if (! robot) return Status::Running;
    if (goToBall||goBehindBall) {
        if (! ball) return Status::Running;
    }
    if (goToBall) {
        targetPos = ball->pos;
    }
    else if (goBehindBall) {
        if (coach::Coach::isRobotBehindBallToGoal(distanceBehindBall, false, robot->pos)){
            return Status::Success;
        }

        auto enemyGoal = Field::get_their_goal_center();
        Vector2 ballToEnemyGoal = enemyGoal - ball->pos;
        Vector2 normalizedBTEG = ballToEnemyGoal.stretchToLength(-distanceBehindBall);
        targetPos = normalizedBTEG + ball->pos;
    }

    // See if the progress is a failure
    if (currentProgress == Progression::FAIL) {
        return Status::Failure;
    }
    deltaPos = targetPos-robot->pos;
    // Now check the progress we made
    currentProgress = checkProgression();
    // Send a move command
    sendMoveCommand2();

    switch (currentProgress) {

        // Return the progression in terms of Status
        case ON_THE_WAY:
            return Status::Running;
        case DONE:
            return Status::Success;
        case FAIL:
            return Status::Failure;
    }

    return Status::Failure;
}

void GoToPos::onTerminate(Status s) {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.w = static_cast<float>(deltaPos.angle());

    command.x_vel = 0;
    command.y_vel = 0;

    publishRobotCommand(command);
}

/// Check if the vector is a valid one
bool GoToPos::checkTargetPos(Vector2 pos) {
    // TODO: actually check
    return true;
}

/// Send a move robot command with a vector
void GoToPos::sendMoveCommand() {

    if (! checkTargetPos(targetPos)) {
        ROS_ERROR("Target position is not correct GoToPos");
        return;
    }

    // TODO: get correct kp from 20-sim model
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 0;

    auto angularVel = (float) Control::calculateAngularVelocity(robot->angle, deltaPos.angle());
    command.w = angularVel;

    command.x_vel = 1.5;// abs(angularVel)/(abs(angularVel)-1);
    command.y_vel = 0;
    publishRobotCommand(command);
    //commandSend = true;
}

/// Send a move robot command with a vector
void GoToPos::sendMoveCommand2() {
    if (! checkTargetPos(targetPos)) {
        ROS_ERROR("Target position is not correct GoToPos");
        return;
    }
    // TODO: get correct kp from 20-sim model
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;

    command.w = static_cast<float>(deltaPos.angle());
    Vector2 deltaPosUnit = deltaPos.normalize();

    command.x_vel = static_cast<float>(deltaPosUnit.x*speed);// abs(angularVel)/(abs(angularVel)-1);
    command.y_vel = static_cast<float>(deltaPosUnit.y*speed);
    publishRobotCommand(command);
    commandSend = true;
}

/// Check the progress the robot made a9nd alter the currentProgress
GoToPos::Progression GoToPos::checkProgression() {
    double maxMargin = 0.2;                        // max offset or something.
    if (deltaPos.length() >= maxMargin) return ON_THE_WAY;
    else return DONE;
}

} // ai
} // rtt