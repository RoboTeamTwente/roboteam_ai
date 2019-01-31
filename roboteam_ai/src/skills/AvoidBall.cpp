//
// Created by mrlukasbos on 24-1-19.
//

#include "AvoidBall.h"
#include "../utilities/Coach.h"
#include "../control/ControlUtils.h"

namespace rtt {
namespace ai {



AvoidBall::AvoidBall(std::string name, bt::Blackboard::Ptr blackboard)
: Skill(std::move(name), std::move(blackboard)) { }

bt::Node::Status AvoidBall::onUpdate() {
    auto robotPos = rtt::Vector2(robot->pos);
    Vector2 force = {0, 0};

    // forces from robots
    for (auto otherRobot : World::getAllRobots()) {
        double distance = robotPos.dist(otherRobot.pos);
        Vector2 distanceVector = robotPos - otherRobot.pos;

        if (otherRobot.id != robot->id && distance < minRobotDistanceForForce) {
            force = force + distanceVector.stretchToLength(1) * (robotWeight/(distance * distance));
        }
    }

    // check forces from ball
    double distance = robotPos.dist(ball->pos);
    if (distance < minBallDistanceForForce) {
        Vector2 distanceVector = robotPos-ball->pos;
        force = force+distanceVector.stretchToLength(1)*(ballWeight/(distance*distance));
    }

    // forces from walls
    double boundWidth =  Field::get_field().boundary_width;
    double halfFieldLength = Field::get_field().field_length/2;
    double halfFieldWidth = Field::get_field().field_width/2;

    std::vector<Vector2> walls;
    walls.emplace_back(Vector2(robotPos.x - halfFieldLength - boundWidth, 0));
    walls.emplace_back(Vector2(robotPos.x + halfFieldLength + boundWidth, 0));
    walls.emplace_back(Vector2(0, robotPos.y - halfFieldWidth - boundWidth));
    walls.emplace_back(Vector2(0, robotPos.y + halfFieldWidth + boundWidth));

    for (auto const &wallVector : walls) {
        if (wallVector.length() < minWallDistanceForForce) {
            force = force + wallVector.stretchToLength(1) * (wallWeight/(wallVector.length() * wallVector.length()));
        }
    }

    // limit the forces
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