//
// Created by thijs on 17-12-18.
//

#include "SideAttacker.h"

namespace rtt {
namespace ai {

std::vector<SideAttacker::RobotPtr> SideAttacker::robotsPositioning = {};
int SideAttacker::robotsInMemory = robotsPositioning.size();

SideAttacker::SideAttacker(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

void SideAttacker::onInitialize() {
    for (auto &robotPositioning : robotsPositioning) {
        if (robotPositioning->id == robot->id) {
            return;
        }
    }
    robotsPositioning.emplace_back(robot);
    robotsInMemory ++;
}

/// Get an update on the skill
bt::Node::Status SideAttacker::onUpdate() {
    bool isInRobotsPositioning = false;
    for (auto &robotPositioning : robotsPositioning) {
        if (robotPositioning->id == robot->id) {
            isInRobotsPositioning = true;
        }
    }

    if (! isInRobotsPositioning) return Status::Running;

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
    std::vector<Vector2> targetLocations = coach::g_offensiveCoach.getNewOffensivePositions(robotsPositioning.size());
    Vector2 position;

    if (zone == - 1 || zone > robotsPositioning.size() - 1 || robotsInMemory != robotsPositioning.size()) {
        std::vector<Vector2> robotLocations;
        std::vector<int> robotIds;

        for (auto &robotPositioning : robotsPositioning) {
            robotIds.push_back(robotPositioning->id);
        }

        rtt::HungarianAlgorithm hungarian;
        map<int, Vector2> shortestDistances;
        shortestDistances = hungarian.getRobotPositions(robotIds, true, targetLocations);

        zone = std::find(targetLocations.begin(), targetLocations.end(), position) - targetLocations.begin() - 1;
        position = shortestDistances[robot->id];
    }
    else {
        position = targetLocations.at(zone);
    }

    robotsInMemory = robotsPositioning.size();
    return position;

}

void SideAttacker::onTerminate(Status s) {
    command.w = static_cast<float>(robot->angle);
    command.x_vel = 0;
    command.y_vel = 0;
    int robotsPositioningSize = robotsPositioning.size();

    for (int i = 0; i < robotsPositioning.size(); i ++) {
        if (robotsPositioning.at(i)->id == robot->id) {
            robotsPositioning.erase(robotsPositioning.begin() + i);
        }
    }

    if (robotsPositioningSize == robotsPositioning.size()) {
        std::cerr << "Robot failed to be removed from robotsPositioning" << std::endl;
    }

    robotsInMemory --;

    zone = - 1;

    publishRobotCommand();
}

} // ai
} // rtt