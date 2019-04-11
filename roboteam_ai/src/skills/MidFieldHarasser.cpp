//
// Created by thijs on 10-4-19.
//

#include "MidFieldHarasser.h"

namespace rtt {
namespace ai {

std::vector<MidFieldHarasser::RobotPtr> MidFieldHarasser::midFieldHarassers = {};
int MidFieldHarasser::robotsInMemory = midFieldHarassers.size();

MidFieldHarasser::MidFieldHarasser(string name, bt::Blackboard::Ptr blackboard)
        : Skill(std::move(name), std::move(blackboard)) {
}

void MidFieldHarasser::onInitialize() {
    for (auto &mFHarasser : midFieldHarassers) {
        if (mFHarasser->id == robot->id) {
            return;
        }
    }
    midFieldHarassers.emplace_back(robot);
    robotsInMemory ++;
}

Skill::Status MidFieldHarasser::onUpdate() {
    bool isInPositioning = false;
    for (auto &robotPositioning : midFieldHarassers) {
        if (robotPositioning->id == robot->id) {
            isInPositioning = true;
        }
    }

    if (! isInPositioning) return Status::Running;


    targetPos = getHarassPosition();

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


Vector2 MidFieldHarasser::getHarassPosition() {
    //std::vector<Vector2> targetLocations = coach::g_offensiveCoach.getNewOffensivePositions(robotsPositioning.size());
    std::vector<Vector2> targetLocations;
    Vector2 position;

    if (zone == - 1 || zone > midFieldHarassers.size() - 1 || robotsInMemory != midFieldHarassers.size()) {
        std::vector<Vector2> robotLocations;
        std::vector<int> robotIds;

        for (auto &mFHarasser : midFieldHarassers) {
            robotIds.push_back(mFHarasser->id);
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

    robotsInMemory = midFieldHarassers.size();
    return position;

}


void MidFieldHarasser::onTerminate(Skill::Status s) {
    command.w = static_cast<float>(robot->angle);
    command.x_vel = 0;
    command.y_vel = 0;
    int size = midFieldHarassers.size();

    for (int i = 0; i < midFieldHarassers.size(); i ++) {
        if (midFieldHarassers.at(i)->id == robot->id) {
            midFieldHarassers.erase(midFieldHarassers.begin() + i);
        }
    }

    if (size == midFieldHarassers.size()) {
        std::cerr << "Robot failed to be removed from midFieldHarassers" << std::endl;
    }

    robotsInMemory --;

    zone = - 1;

    publishRobotCommand();
}

}
}