//
// Created by mrlukasbos on 24-1-19.
//

#include "AvoidBall.h"
#include "../control/ControlUtils.h"
#include <cmath>
#include <roboteam_ai/src/coach/BallplacementCoach.h>
#include "../world/Field.h"

namespace rtt {
namespace ai {

using cu = control::ControlUtils;

AvoidBall::AvoidBall(std::string name, bt::Blackboard::Ptr blackboard)
: Skill(std::move(name), std::move(blackboard)) {
    stop = properties->getBool("Stop");
    if(stop) minRobotDistanceForForce *= 1.5;
}

bt::Node::Status AvoidBall::onUpdate() {
    auto robotPos = rtt::Vector2(robot->pos);
    Vector2 force = {0, 0};

    // forces from robots
    for (auto &otherRobot : world::world->getAllRobots()) {
        if (otherRobot.id != robot->id) {
            force = force + cu::calculateForce(robotPos - otherRobot.pos, robotWeight, minRobotDistanceForForce);
        }
    }
    // check forces from ball
    force = force + cu::calculateForce(robotPos - ball->pos, ballWeight, minBallDistanceForForce);

    // forces from walls
    auto field = world::field->get_field();
    double boundWidth =  field.boundary_width;
    double halfFieldLength = field.field_length/2;
    double halfFieldWidth = field.field_width/2;

    std::vector<Vector2> wallsVectors;
    wallsVectors.emplace_back(Vector2(robotPos.x - halfFieldLength - boundWidth, 0));
    wallsVectors.emplace_back(Vector2(robotPos.x + halfFieldLength + boundWidth, 0));
    wallsVectors.emplace_back(Vector2(0, robotPos.y - halfFieldWidth - boundWidth));
    wallsVectors.emplace_back(Vector2(0, robotPos.y + halfFieldWidth + boundWidth));

    for (auto const &wallVector : wallsVectors) {
        force = force + cu::calculateForce(wallVector, wallWeight, minWallDistanceForForce);
    }

    // https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_placement_interference
    Vector2 bpTarget = coach::g_ballPlacement.getBallPlacementPos();
    // if the robot is closer to the ballplacementTarget than the ball
    if (control::ControlUtils::distanceToLineWithEnds(robot->pos, ball->pos, bpTarget) < minBallDistanceForForce) {
        Vector2 LineToBallPlacementBallLine = robot->pos - robot->pos.project(bpTarget, ball->pos);
        force = force + cu::calculateForce(LineToBallPlacementBallLine, ballWeight, minBallDistanceForForce);
    }

    force = control::ControlUtils::velocityLimiter(force, Constants::MAX_VEL_BALLPLACEMENT());
    command.use_angle = 1;
    command.w = static_cast<float>(force.angle());
    command.x_vel = static_cast<float>(force.x);
    command.y_vel = static_cast<float>(force.y);

    publishRobotCommand();

    return Status::Running;
}

} // ai
} // rtt