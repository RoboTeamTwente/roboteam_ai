//
// Created by thijs on 17-12-18.
//

#include "SideAttacker.h"

namespace rtt {
namespace ai {

std::vector<SideAttacker::RobotPtr> SideAttacker::robotsPositioning = {};

SideAttacker::SideAttacker(string name, bt::Blackboard::Ptr blackboard)
    :Skill(std::move(name), std::move(blackboard)) {
}

void SideAttacker::onInitialize() {
    for (auto & i : robotsPositioning) {
        if (i->id == robot->id) {
            return;
        }
    } // TODO use std::find here, love best teammate ever

    robotsPositioning.emplace_back(robot);
}


/// Get an update on the skill
bt::Node::Status SideAttacker::onUpdate() {

    bool isInRobotsPositioning = false;
    for (auto & robotPositioning : robotsPositioning) {
        if (robotPositioning->id == robot->id) {
            isInRobotsPositioning = true;
        }
    }

    if (!isInRobotsPositioning) return Status::Running;

    targetPos = getOffensivePosition();
    auto newPosition = goToPos.getPosVelAngle(robot, targetPos);
    Vector2 velocity = newPosition.vel;
    velocity = control::ControlUtils::velocityLimiter(velocity);
    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    command.w = static_cast<float>(newPosition.angle);

    command.use_angle = 1;
    publishRobotCommand();

    return Status::Running;
}

Vector2 SideAttacker::getOffensivePosition() {
    auto field = world::field->get_field();

    std::vector<Vector2> targetLocations = coach::g_offensiveCoach.getNewOffensivePositions(robotsPositioning.size());

    // If not yet assigned to a zone, do so according to the Hungarian
    if (zone == -1) {
        std::vector<Vector2> robotLocations;
        std::vector<int> robotIds;

        for (auto & robotPositioning : robotsPositioning) {
            robotIds.push_back(robotPositioning->id);
        }

        rtt::HungarianAlgorithm hungarian;
        auto shortestDistances = hungarian.getRobotPositions(robotIds, true, targetLocations);
        return shortestDistances.at(robot->id);

    // If already assigned to a zone, stick to it
    } else {
        return targetLocations.at(zone);
    }
}

void SideAttacker::onTerminate(Status s) {
    command.w = static_cast<float>(deltaPos.angle());
    command.x_vel = 0;
    command.y_vel = 0;
    publishRobotCommand();
}

} // ai
} // rtt