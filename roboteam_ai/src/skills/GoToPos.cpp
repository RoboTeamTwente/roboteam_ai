//
// Created by baris on 24/10/18.
//

#include "GoToPos.h"

namespace rtt {
namespace ai {

GoToPos::GoToPos(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {
}

/// Init the GoToPos skill
void GoToPos::onInitialize() {
    robot = getRobotFromProperties(properties);

    goToBall = properties->getBool("goToBall");
    goBehindBall = properties->getBool("goBehindBall");

    if (properties->hasVector2("Position")) {
        targetPos = properties->getVector2("Position");
    }
    else {
        ROS_ERROR("GoToPos Initialize -> No good X or Y set in properties");
        currentProgress = Progression::FAIL;
    }

}

/// Get an update on the skill
bt::Node::Status GoToPos::onUpdate() {
    updateRobot();
    if (!robot) return Status::Running;

    if (goToBall) {
        auto ball = World::getBall();
        targetPos = ball.pos;
    } else if (goBehindBall) {
        auto ball = World::getBall();
        auto enemyGoal = Field::get_their_goal_center();
        auto ballToEnemyGoal = enemyGoal - ball.pos;
        auto normalizedBTEG = ballToEnemyGoal.normalize();
        targetPos = {ball.pos.x - normalizedBTEG.x, ball.pos.y - normalizedBTEG.y};
    }

    // See if the progress is a failure
    if (currentProgress == Progression::FAIL) {
        return status::Failure;
    }
    double dx = targetPos.x - robot->pos.x;
    double dy = targetPos.y - robot->pos.y;
    deltaPos = {dx, dy};

    // Now check the progress we made
    currentProgress = checkProgression();
    // Send a move command
    sendMoveCommand2();

    switch (currentProgress) {

        // Return the progression in terms of status
    case ON_THE_WAY:return status::Running;
    case DONE: return status::Success;
    case FAIL: return status::Failure;
    }

    return status::Failure;
}

void GoToPos::onTerminate(status s) {
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

    auto angularVel = (float)Control::calculateAngularVelocity(robot->angle, deltaPos.angle());
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

    command.w= static_cast<float>(deltaPos.angle());
    Vector2 deltaPosUnit=deltaPos.normalize();

    command.x_vel = (float) deltaPosUnit.x*2;// abs(angularVel)/(abs(angularVel)-1);
    command.y_vel = (float) deltaPosUnit.y*2;
    publishRobotCommand(command);
    commandSend = true;
}

/// Check the progress the robot made a9nd alter the currentProgress
GoToPos::Progression GoToPos::checkProgression() {
    double maxMargin = 0.15;                        // max offset or something.
    if (deltaPos.length() >= maxMargin) return ON_THE_WAY;
    else return DONE;
}

} // ai
} // rtt