//
// Created by rolf on 10/12/18.
//

#include <roboteam_ai/src/interface/drawer.h>
#include <roboteam_ai/src/control/Controller.h>
#include "Keeper.h"
namespace rtt {
namespace ai {
Keeper::Keeper(rtt::string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

void Keeper::onInitialize() {

    goalPos = Field::get_our_goal_center();
    goalwidth = Field::get_field().goal_width;

    //Create arc for keeper to drive on
    blockCircle=control::ControlUtils::createKeeperArc();
    //TODO::magic numbers galore, from the old team. move to new control library
    double timediff = 1.0/constants::tickRate;
    pid.setPID(3,0.0, 0.2, timediff);
    finePid.setPID(1.0,0.0,0.0,timediff);
}

Keeper::Status Keeper::onUpdate() {
        Vector2 ballPos = World::getBall().pos;
        Vector2 blockPoint = computeBlockPoint(ballPos);
        //double dist=control::ControlUtils::distanceToLine(robot->pos,ballPos,blockPoint);
        double dist = (blockPoint - (Vector2(robot->pos))).length(); //using point distance not line distance.
        if (dist < constants::KEEPER_POSDIF) {
            sendStopCommand();
        }
        else if (dist <2*constants::ROBOT_RADIUS){
            sendFineMoveCommand(blockPoint);
        }
        else {
            sendMoveCommand(blockPoint);
        }
        return Status::Running;
}

void Keeper::onTerminate(Status s) {
    roboteam_msgs::RobotCommand cmd;
    cmd.use_angle = 1;
    cmd.id = robotId;
    cmd.x_vel = 0;
    cmd.y_vel = 0;
    cmd.w = static_cast<float>(M_PI_2);
    publishRobotCommand(cmd);
}

void Keeper::sendMoveCommand(Vector2 pos) {
    Vector2 error = pos - robot->pos;
    Vector2 delta = pid.controlPIR(error, robot->vel);
    Vector2 deltaLim=control::ControlUtils::VelocityLimiter(delta);
    roboteam_msgs::RobotCommand cmd;
    cmd.use_angle = 1;
    cmd.id = robot->id;
    cmd.x_vel = static_cast<float>(deltaLim.x);
    cmd.y_vel = static_cast<float>(deltaLim.y);
    cmd.w = static_cast<float>(M_PI_2);
    publishRobotCommand(cmd);
}
void Keeper::sendFineMoveCommand(Vector2 pos) {
    Vector2 error = pos - robot->pos;
    Vector2 delta = finePid.controlPIR(error, robot->vel);
    Vector2 deltaLim=control::ControlUtils::VelocityLimiter(delta);
    roboteam_msgs::RobotCommand cmd;
    cmd.use_angle = 1;
    cmd.id = robot->id;
    cmd.x_vel = static_cast<float>(deltaLim.x);
    cmd.y_vel = static_cast<float>(deltaLim.y);
    cmd.w = static_cast<float>(M_PI_2);
    publishRobotCommand(cmd);
}

void Keeper::sendStopCommand() {
    roboteam_msgs::RobotCommand cmd;
    cmd.use_angle = 1;
    cmd.id = robot->id;
    cmd.x_vel = static_cast<float>(0.0);
    cmd.y_vel = static_cast<float>(0.0);
    cmd.w = static_cast<float>(M_PI_2);
    publishRobotCommand(cmd);
}
Vector2 Keeper::computeBlockPoint(Vector2 defendPos) {
    Vector2 u1 = (goalPos + Vector2(0.0, goalwidth*0.5) - defendPos).normalize();
    Vector2 u2 = (goalPos + Vector2(0.0, - goalwidth*0.5) - defendPos).normalize();
    double dist = (defendPos - goalPos).length();
    Vector2 blockLineStart = defendPos + (u1 + u2).stretchToLength(dist);
    std::pair<boost::optional<Vector2>, boost::optional<Vector2>> intersections = blockCircle.intersectionWithLine(
            blockLineStart, defendPos);
    Vector2 blockPos, posA, posB;
    // go stand on the intersection of the lines. Pick the one that is closest to (0,0) if there are multiple
    if (intersections.first && intersections.second) {
        posA = *intersections.first;
        posB = *intersections.second;
        if (posA.length() < posB.length()) {
            blockPos = posA;
        }
        else blockPos = posB;
    }
    else if (intersections.first) {
        blockPos = *intersections.first;
    }
    else if (intersections.second) {
        blockPos = *intersections.second;
    }
    else {
        blockPos = Vector2(goalPos.x + constants::KEEPER_POST_MARGIN, goalwidth/2
                *signum(defendPos.y)); // Go stand at one of the poles depending on the side the defendPos is on.
    }
    //Interface visualization:
    std::vector<std::pair<rtt::Vector2, QColor>> displayColorData;
    std::pair<rtt::Vector2, QColor> A=std::make_pair(blockPos,Qt::red);
    displayColorData.push_back(A);
    displayColorData.emplace_back(std::make_pair(blockLineStart,Qt::red));
    displayColorData.emplace_back(std::make_pair(defendPos,Qt::red));
    displayColorData.emplace_back(std::make_pair(goalPos + Vector2(0.0, goalwidth*0.5),Qt::green));
    displayColorData.emplace_back(std::make_pair(goalPos - Vector2(0.0, goalwidth*0.5),Qt::green));
    displayColorData.emplace_back(std::make_pair(robot->pos,Qt::blue));
    interface::Drawer::setKeeperPoints(robot->id,displayColorData);

    return blockPos;
}

}
}