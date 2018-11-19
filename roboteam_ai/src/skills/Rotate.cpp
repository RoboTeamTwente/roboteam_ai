//
// Created by thijs on 24-10-18.
//

#include "Rotate.h"
#include "math.h"

namespace rtt {
namespace ai {

Rotate::Rotate(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {
}

/// Init the Rotate skill
void Rotate::Initialize() {

    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robot.id = (unsigned int) RobotDealer::findRobotForRole(roleName);
        if (World::getRobotForId(robot.id, true)) {
            robot = World::getRobotForId(robot.id, true).get();
        }
        else {
            ROS_ERROR("Rotate Initialize -> robot does not exist in world");
            currentProgress = Progression::INVALID;
            return;
        }
    }
    else {
        ROS_ERROR("Rotate Initialize -> ROLE INVALID!!");
        currentProgress = Progression::FAIL;
        return;
    }

//  ____________________________________________________

    rotateToBall = properties->getBool("rotateToBall");
    rotateToOurGoal = properties->getBool("rotateToOurGoal");
    rotateToEnemyGoal = properties->getBool("rotateToEnemyGoal");
    rotateToRobotID = properties->getInt("rotateToRobotID");
    robotIsEnemy = properties->getBool("robotIsEnemy");

    if (properties->hasDouble("Angle")) {
        targetAngle = properties->getDouble("Angle");
    }
    else {
        ROS_ERROR("Rotate Initialize -> No good angle set in properties");
        currentProgress = Progression::FAIL;
    }

}

bt::Node::Status Rotate::Update() {

    if (World::getRobotForId(robot.id, true)) {
        robot = World::getRobotForId(robot.id, true).get();
    }
    else {
        ROS_ERROR("Rotate Update -> robot does not exist in world");
        currentProgress = Progression::INVALID;
    }

    if (rotateToBall) {
        auto ball = World::getBall();
        Vector2 deltaPos = {ball.pos.x - robot.pos.x, ball.pos.y - robot.pos.y};
        targetAngle = deltaPos.angle();

    }
    else if (rotateToEnemyGoal) {
        auto enemyGoal = Field::get_their_goal_center();
        Vector2 deltaPos = {enemyGoal.x - robot.pos.x, enemyGoal.y - robot.pos.y};
        targetAngle = deltaPos.angle();

    }
    else if (rotateToEnemyGoal) {
        auto ourGoal = Field::get_their_goal_center();
        Vector2 deltaPos = {ourGoal.x - robot.pos.x, ourGoal.y - robot.pos.y};
        targetAngle = deltaPos.angle();

    }
    else if (rotateToRobotID != - 1) {
        if (robotIsEnemy) {
            if (World::getRobotForId(rotateToRobotID, false)) {
                auto otherRobot = World::getRobotForId(rotateToRobotID, false).get();
                Vector2 deltaPos = {otherRobot.pos.x - robot.pos.x, otherRobot.pos.y - robot.pos.y};
                targetAngle = deltaPos.angle();

            }
        }
        else {
            if (World::getRobotForId(rotateToRobotID, true)) {
                auto otherRobot = World::getRobotForId(rotateToRobotID, true).get();
                Vector2 deltaPos = {otherRobot.pos.x - robot.pos.x, otherRobot.pos.y - robot.pos.y};
                targetAngle = deltaPos.angle();
            }
        }
    }

    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    command.use_angle = 1;

    command.w = (float) Control::calculateAngularVelocity(robot.angle, targetAngle);
    publishRobotCommand(command);
    std::cerr << "Rotate command -> id: " << command.id << ", w_vel: " << command.w << std::endl;
    currentProgress = checkProgression();

    switch (currentProgress) {
    case ROTATING: return Status::Running;
    case DONE: return Status::Success;
    case FAIL: return Status::Failure;
    case INVALID: return Status::Invalid;
    }

    return Status::Failure;
}

void Rotate::Terminate(Status s) {
    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    command.use_angle = 1;
    command.w = 0.0f;
    publishRobotCommand(command);
}

std::string Rotate::node_name() {
    return "Rotate";
}

Rotate::Progression Rotate::checkProgression() {

    double errorMargin = 0.05;
    if (deltaAngle > errorMargin) return ROTATING;
    else return DONE;
}


} // ai
} // rtt
