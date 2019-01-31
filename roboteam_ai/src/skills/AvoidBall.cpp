//
// Created by mrlukasbos on 24-1-19.
//

#include "AvoidBall.h"
#include "../utilities/Coach.h"
#include "../control/ControlUtils.h"
#include <cmath>

namespace rtt {
namespace ai {

using cu = control::ControlUtils;

AvoidBall::AvoidBall(std::string name, bt::Blackboard::Ptr blackboard)
: Skill(std::move(name), std::move(blackboard)) { }

bt::Node::Status AvoidBall::onUpdate() {
    auto robotPos = rtt::Vector2(robot->pos);
    Vector2 force = {0, 0};

    // forces from robots
    for (auto otherRobot : World::getAllRobots()) {
        if (otherRobot.id != robot->id) {
            force = force + cu::calculateForce(robotPos - otherRobot.pos, constants::robotWeight, constants::minRobotDistanceForForce);
        }
    }
    // check forces from ball
    force = force + cu::calculateForce(robotPos - ball->pos, constants::ballWeight, constants::minBallDistanceForForce);

    // forces from walls
    auto field = Field::get_field();
    double boundWidth =  field.boundary_width;
    double halfFieldLength = field.field_length/2;
    double halfFieldWidth = field.field_width/2;

    std::vector<Vector2> wallsVectors;
    wallsVectors.emplace_back(Vector2(robotPos.x - halfFieldLength - boundWidth, 0));
    wallsVectors.emplace_back(Vector2(robotPos.x + halfFieldLength + boundWidth, 0));
    wallsVectors.emplace_back(Vector2(0, robotPos.y - halfFieldWidth - boundWidth));
    wallsVectors.emplace_back(Vector2(0, robotPos.y + halfFieldWidth + boundWidth));

    for (auto const &wallVector : wallsVectors) {
        force = force + cu::calculateForce(wallVector, constants::wallWeight, constants::minWallDistanceForForce);
    }

    // limit the forces
    // TODO do not always limit the speed for ballplacement only
    if (force.length() > constants::MAX_VEL_BALLPLACEMENT) force.stretchToLength(constants::MAX_VEL_BALLPLACEMENT);
    if (force.angle() > constants::MAX_ANGULAR_VELOCITY) force.stretchToLength(constants::MAX_ANGULAR_VELOCITY);

    roboteam_msgs::RobotCommand command;
    if (force.length() < 0.2) {
        force = {0, 0};
        command.use_angle = 0;
        command.w = 0;
    } else {
        command.use_angle = 1;
        command.w = static_cast<float>(force.angle());
    }

    command.id = robot->id;
    command.x_vel = static_cast<float>(force.x);
    command.y_vel = static_cast<float>(force.y);
    publishRobotCommand(command);
}

} // ai
} // rtt