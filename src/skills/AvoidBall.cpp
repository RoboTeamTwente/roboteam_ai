//
// Created by mrlukasbos on 24-1-19.
//

#include "skills/AvoidBall.h"
#include "control/ControlUtils.h"
#include <cmath>
#include <coach/BallplacementCoach.h>
#include <interface/api/Input.h>
#include "world/FieldComputations.h"
#include "control/numtrees/NumTreePosControl.h"
#include "utilities/RobotDealer.h"

namespace rtt::ai {

using cu = control::ControlUtils;

AvoidBall::AvoidBall(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void AvoidBall::onInitialize(std::string type) {
    minRobotDistanceForForce = 0.9;
    stop = properties->getBool("Stop");
    if (stop) minRobotDistanceForForce = 0.7 * 1.5;
    // type = stringToType(properties->getString("type"));
    this->type = stringToType(type);

    if (this->type == PASSING) {
        receiver = world->getRobotForId(coach::g_pass.getRobotBeingPassedTo(), true);
    }
}

bt::Node::Status AvoidBall::onUpdate() {
    auto robotPos = rtt::Vector2(robot->pos);

    bool robotIsKeeper = (robotDealer::RobotDealer::keeperExistsInWorld() && robot->id == robotDealer::RobotDealer::getKeeperID());
    if (!robotIsKeeper && (FieldComputations::pointIsInDefenceArea(*field, robotPos, true, 0.10) ||
        FieldComputations::pointIsInDefenceArea(*field, robotPos, false, 0.10))) {

        robot->getNumtreePosControl()->getRobotCommand(world, field, robot, Vector2(0, robotPos.y));
        publishRobotCommand();
        return Status::Running;
    }

    Vector2 force = {0, 0};

    // forces from robots
    for (auto &otherRobot : world::world->getAllRobots()) {
        if (otherRobot->id != robot->id) {
            force = force + cu::calculateForce(robotPos - otherRobot->pos, robotWeight, minRobotDistanceForForce);
        }
    }
    // check forces from ball
    force = force + cu::calculateForce(robotPos - ball->getPos(), ballWeight, minBallDistanceForForce);

    // forces from walls
    double boundWidth = (*field).getBoundaryWidth();
    double halfFieldLength = (*field).getFieldLength() / 2;
    double halfFieldWidth = (*field).getFieldWidth() / 2;

    std::vector<Vector2> wallsVectors;
    wallsVectors.emplace_back(Vector2(robotPos.x - halfFieldLength - boundWidth, 0));
    wallsVectors.emplace_back(Vector2(robotPos.x + halfFieldLength + boundWidth, 0));
    wallsVectors.emplace_back(Vector2(0, robotPos.y - halfFieldWidth - boundWidth));
    wallsVectors.emplace_back(Vector2(0, robotPos.y + halfFieldWidth + boundWidth));

    for (auto const &wallVector : wallsVectors) {
        force = force + cu::calculateForce(wallVector, wallWeight, minWallDistanceForForce);
    }

    if (type == BALLPLACEMENT) {
        Vector2 bpTarget = coach::g_ballPlacement.getBallPlacementPos();
        // if the robot is closer to the ballplacementTarget than the ball
        if (control::ControlUtils::distanceToLineWithEnds(robot->pos, ball->getPos(), bpTarget) < minBallDistanceForForce) {
            Vector2 LineToBallPlacementBallLine = robot->pos - robot->pos.project(bpTarget, ball->getPos());
            force = force + cu::calculateForce(LineToBallPlacementBallLine, ballWeight, minBallDistanceForForce);
        }
    }

    if (type == PASSING) {
        // if robot's projection is on the pass line
        if (control::ControlUtils::isPointProjectedOnLineSegment(robot->pos, ball->getPos(), receiver->pos)) {
            Vector2 projectionOnPassLine = robot->pos.project(ball->getPos(), receiver->pos);
            Vector2 distanceToProjection = robot->pos - projectionOnPassLine;
            force = force + cu::calculateForce(distanceToProjection, ballWeight, minBallDistanceForForce);
        }
    }

    command.set_use_angle(true);
    command.set_w(static_cast<float>(force.angle()));
    command.mutable_vel()->set_x(static_cast<float>(force.x));
    command.mutable_vel()->set_y(static_cast<float>(force.y));

    publishRobotCommand();

    return Status::Running;
}

AvoidBall::Type AvoidBall::stringToType(std::string string) {
    if (string == "ballPlacement") {
        return BALLPLACEMENT;
    } else if (string == "passing") {
        return PASSING;
    } else {
        return BALLPLACEMENT;
    }
}

}  // namespace rtt::ai