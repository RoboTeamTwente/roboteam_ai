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
void GoToPos::Initialize() {

    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robot.id = (unsigned int) RobotDealer::findRobotForRole(roleName);
        if (World::getRobotForId(robot.id, true)) {
            robot = World::getRobotForId(robot.id, true).get();
        }
        else {
            ROS_ERROR("GoToPos Initialize -> robot does not exist in world");
            currentProgress = Progression::INVALID;
            return;
        }
    }
    else {
        ROS_ERROR("GoToPos Initialize -> ROLE INVALID!!");
        currentProgress = Progression::INVALID;
        return;
    }
//  ____________________________________________________

    goToBall = properties->getBool("goToBall");
    goBehindBall = properties->getBool("goBehindBall");

    if (properties->hasVector2("Position")) {
        targetPos = properties->getVector2("Position");
    }
    else {
        ROS_ERROR("GoToPos Initialize -> No good X or Y set in properties");
        currentProgress = Progression::INVALID;
    }

}

/// Get an update on the skill
bt::Node::Status GoToPos::Update() {

    if (World::getRobotForId(robot.id, true)) {
        robot = World::getRobotForId(robot.id, true).get();
    } else {
        ROS_ERROR("GoToPos Update -> robot does not exist in world");
        currentProgress = Progression::FAIL;
    }

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
    else if (currentProgress == Progression::INVALID) {
        return status::Invalid;
    }

    // Now check the progress we made
    currentProgress = checkProgression();
    // Send a move command
    sendMoveCommand();

    switch (currentProgress) {

        // Return the progression in terms of status
    case ON_THE_WAY:return status::Running;
    case DONE: return status::Success;
    case FAIL: return status::Failure;
    case INVALID: return status::Invalid;
    }

    return status::Failure;
}

void GoToPos::Terminate(status s) {

    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    command.use_angle = 1;
    command.w = 0;

    command.x_vel = 0;
    command.y_vel = 0;

    publishRobotCommand(command);
    commandSend = true;

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
    command.id = robot.id;
    command.use_angle = 1;
    auto angularVel = (float) getAngularVelocity();
    command.w = angularVel;

    command.x_vel = 1.5;// abs(angularVel)/(abs(angularVel)-1);
    command.y_vel = 0;
    publishRobotCommand(command);
    commandSend = true;
    std::cerr << "GoToPos command -> id: " << command.id << ", xvel: " << command.x_vel << ", yvel: " << command.y_vel
              << ", w_vel: " << command.w
              << std::endl;
}

/// Check the progress the robot made and alter the currentProgress
GoToPos::Progression GoToPos::checkProgression() {

    double dx = targetPos.x - robot.pos.x;
    double dy = targetPos.y - robot.pos.y;
    deltaPos = {dx, dy};

    double maxMargin = 0.2;                        // max offset or something.

    if (deltaPos.length() >= maxMargin) return ON_THE_WAY;
    else return DONE;
}


std::string GoToPos::node_name() {
    return "GoToPos";
}

double GoToPos::getAngularVelocity() {

    double direction = 1;               // counter clockwise rotation

    auto targetAngle = (float) deltaPos.angle();

    float angleDiff = targetAngle - robot.angle;
    while (angleDiff < 0) angleDiff += 2*M_PI;
    while (angleDiff > 2*M_PI) angleDiff -= 2*M_PI;
    if (angleDiff > M_PI) {
        angleDiff = (float) (2.0*M_PI - angleDiff);
        direction = - 1;                //  clockwise rotation
    }
    if (angleDiff>1)angleDiff=1;
    return direction*(angleDiff*angleDiff)*MAX_ANGULAR_VELOCITY;



}

} // ai
} // rtt