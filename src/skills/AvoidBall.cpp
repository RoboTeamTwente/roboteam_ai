//
// Created by mrlukasbos on 24-1-19.
//

#include <skills/AvoidBall.h>
#include <coach/BallplacementCoach.h>
#include <utilities/RobotDealer.h>

namespace rtt::ai {

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
    auto robotPos = rtt::Vector2(robot->get()->getPos());

    bool robotIsKeeper = (robotDealer::RobotDealer::keeperExistsInWorld() && robot->get()->getId() == robotDealer::RobotDealer::getKeeperID());
    if (!robotIsKeeper && (FieldComputations::pointIsInDefenceArea(*field, robotPos, true, 0.10) || FieldComputations::pointIsInDefenceArea(*field, robotPos, false, 0.10))) {
        robot->getControllers().getNumTreePosController()->getRobotCommand(robot->get()->getId(), Vector2(0, robotPos.y));
        publishRobotCommand();
        return Status::Running;
    }

    Vector2 force = {0, 0};

    // forces from robots
    for (auto &otherRobot : world::world->getAllRobots()) {
        if (otherRobot->id != robot->get()->getId()) {
            force = force + control::ControlUtils::calculateForce(robotPos - otherRobot->pos, robotWeight, minRobotDistanceForForce);
        }
    }
    // check forces from ball
    force = force + control::ControlUtils::calculateForce(robotPos - ball->get()->getPos(), ballWeight, minBallDistanceForForce);

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
        force = force + control::ControlUtils::calculateForce(wallVector, wallWeight, minWallDistanceForForce);
    }

    if (type == BALLPLACEMENT) {
        Vector2 bpTarget = coach::g_ballPlacement.getBallPlacementPos();
        // if the robot is closer to the ballplacementTarget than the ball
        if (control::ControlUtils::distanceToLineWithEnds(robot->get()->getPos(), ball->get()->getPos(), bpTarget) < minBallDistanceForForce) {
            Vector2 LineToBallPlacementBallLine = robot->get()->getPos() - robot->get()->getPos().project(bpTarget, ball->get()->getPos());
            force = force + control::ControlUtils::calculateForce(LineToBallPlacementBallLine, ballWeight, minBallDistanceForForce);
        }
    }

    if (type == PASSING) {
        // if robot's projection is on the pass line
        if (control::ControlUtils::isPointProjectedOnLineSegment(robot->get()->getPos(), ball->get()->getPos(), receiver->get()->getPos())) {
            Vector2 projectionOnPassLine = robot->get()->getPos().project(ball->get()->getPos(), receiver->get()->getPos());
            Vector2 distanceToProjection = robot->get()->getPos() - projectionOnPassLine;
            force = force + control::ControlUtils::calculateForce(distanceToProjection, ballWeight, minBallDistanceForForce);
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