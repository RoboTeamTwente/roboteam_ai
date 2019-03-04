//
// Created by rolf on 10/12/18.
//

#include <roboteam_ai/src/interface/drawer.h>
#include <roboteam_ai/src/control/Controller.h>
#include "Keeper.h"
#include "../utilities/Field.h"

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
    double timediff = 1.0/Constants::TICK_RATE();
    pid.setPID(3.7,1.7,0.8, timediff);
    finePid.setPID(3.7,1.7,0.8,timediff);
}

Keeper::Status Keeper::onUpdate() {
        Vector2 ballPos = World::getBall()->pos;
        Vector2 blockPoint;

        shared_ptr<roboteam_msgs::WorldRobot> closestBot;
        closestBot = World::getRobotClosestToPoint(ballPos, robot->id, 0);
        if(closestBot) {
            double closestBotAngle = closestBot->angle;

            // TODO: HACK HACK DO NOT LOOK AT OUR OWN ROBOTS BUT OPPONENT'S
            if (World::BotHasBall(closestBot->id, true, 0.25) && viewAtGoal(closestBot)) {
                blockPoint = computeBlockPointWithAttacker(closestBot);
                if (!Field::pointIsInField(blockPoint) || !Field::pointIsInDefenceArea(blockPoint, true)) {
                    blockPoint = computeBlockPoint(ballPos);
                }
            } else {
                blockPoint = computeBlockPoint(ballPos);
            }
        } else {
            blockPoint = computeBlockPoint(ballPos);
        }
        if (!Field::pointIsInField(blockPoint, 0.03)) {
            std::cout << "Keeper escaping field!" << std::endl;
            return Status::Running;
        } else {
            Vector2 velocities = goToPos.goToPos(robot, blockPoint);
            velocities = control::ControlUtils::VelocityLimiter(velocities);
            roboteam_msgs::RobotCommand command;
            command.id = robot->id;
            command.x_vel = static_cast<float>(velocities.x);
            command.y_vel = static_cast<float>(velocities.y);
            publishRobotCommand(command);
            return Status::Running;

        }
        //double dist=control::ControlUtils::distanceToLine(robot->pos,ballPos,blockPoint);
        double dist = (blockPoint - (Vector2(robot->pos))).length(); //using point distance not line distance.
        if (dist < Constants::KEEPER_POSDIF()) {
            sendStopCommand();
        }
        else if (dist < 2*Constants::ROBOT_RADIUS()){
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

        if (!Field::pointIsInDefenceArea(posA, true)) {
            blockPos = posB;
        }

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
        blockPos = Vector2(goalPos.x + Constants::KEEPER_POST_MARGIN(), goalwidth/2
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

Vector2 Keeper::computeBlockPointWithAttacker(shared_ptr<roboteam_msgs::WorldRobot> attacker) {
    Vector2 interceptionPoint;

    Vector2 start;
    Vector2 end;
    double distanceToGoal = ((Vector2)attacker->pos - Field::get_our_goal_center()).length();

    start = attacker->pos;
    end = start + (Vector2){distanceToGoal, 0}.rotate(attacker->angle);

    Arc keeperCircle = control::ControlUtils::createKeeperArc();
    std::pair<boost::optional<Vector2>, boost::optional<Vector2>> intersections = keeperCircle.intersectionWithLine(
            start, end);

    if (intersections.first && intersections.second) {
        double dist1 = (Vector2(robot->pos) - *intersections.first).length();
        double dist2 = (Vector2(robot->pos) - *intersections.second).length();
        if (dist2 < dist1) {
            interceptionPoint = *intersections.second;
        }
        else { interceptionPoint = *intersections.first; }
    }
    else if (intersections.first) {
        interceptionPoint = *intersections.first;
    }
    else if (intersections.second) {
        interceptionPoint = *intersections.second;
    }
    else {
        // if the Line does not intercept it usually means the ball is coming from one of the corners-ish to the keeper
        // For now we pick the closest point to the (predicted) line of the ball
        interceptionPoint = Vector2(robot->pos).project(start, end);
    }

    return interceptionPoint;
}

bool Keeper::viewAtGoal(shared_ptr<roboteam_msgs::WorldRobot> attacker) {
    Vector2 start;
    Vector2 end;
    double distanceToGoal = ((Vector2)attacker->pos - Field::get_our_goal_center()).length();

    start = attacker->pos;
    end = start + (Vector2){distanceToGoal * 1.2, 0}.rotate(attacker->angle);

    auto field = Field::get_field();
    Vector2 startGoal = {-field.field_length, -field.goal_width / 2};
    Vector2 endGoal = {-field.field_length, field.goal_width / 2};

    return control::ControlUtils::lineSegmentsIntersect(start, end, startGoal, endGoal);
}

}
}