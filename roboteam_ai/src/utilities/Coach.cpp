//
// Created by baris on 6-12-18.
//

#include "Coach.h"

namespace rtt {
namespace ai {
namespace coach {

std::map<int, int> Coach::defencePairs;

int Coach::pickOffensivePassTarget(int selfID, std::string roleName) {

    // Get the other robots in that tactic
    std::string tacticName = dealer::getTacticNameForRole(std::move(roleName));
    auto tacticMates = dealer::findRobotsForTactic(tacticName);

    // Pick a free one TODO make better
    for (auto bot : tacticMates) {
        if (bot != selfID) {
            if (control::ControlUtils::hasClearVision(selfID, bot, World::get_world(), 2)) {
                return bot;
            }
        }
    }
    return - 1;
}
int Coach::pickDefensivePassTarget(int selfID) {

    auto world = World::get_world();
    auto us = world.us;
    int safelyness = 3;
    while (safelyness > 0) {
        for (auto friendly : us) {
            if (control::ControlUtils::hasClearVision(selfID, friendly.id, world, safelyness)) {
                return friendly.id;
            }
        }
        safelyness --;
    }
    return - 1;
}
int Coach::pickHarassmentTarget(int selfID) {
    auto world = World::get_world();
    auto them = world.them;
    dangerfinder::DangerData dangerData = dangerfinder::DangerFinder::instance().getMostRecentData();
    std::vector<int> dangerList = dangerData.dangerList; // A list of robot IDs, sorted from most to least dangerous

    return *dangerList.begin();
}

int Coach::whichRobotHasBall(bool isOurTeam) {
    roboteam_msgs::World world = World::get_world();
    std::vector<roboteam_msgs::WorldRobot> robots;
    if (isOurTeam) {
        robots = world.us;
    }
    else {
        robots = world.them;
    }

    for (auto &robot:robots) {
        if (doesRobotHaveBall(robot.id, isOurTeam)) {
            return robot.id;
        }
    }
    return - 1;
}

int Coach::doesRobotHaveBall(unsigned int robotID, bool isOurTeam) {
    auto robot = World::getRobotForId(robotID, isOurTeam);
    Vector2 ballPos = World::get_world().ball.pos;

    Vector2 deltaPos = (ballPos - robot->pos);
    double dist = deltaPos.length();
    double angle = deltaPos.angle();
    double robotAngle = robot->angle;

    if (angle < 0) {
        angle += 2*M_PI;
    }
    if (robotAngle < 0) {
        robotAngle += 2*M_PI;
    }

    return ((dist < 0.25) && (fabs(angle - robotAngle) < 0.4));
}

int Coach::pickOpponentToCover(int selfID) {
    dangerfinder::DangerData DangerData = dangerfinder::DangerFinder::instance().getMostRecentData();
    std::vector<int> dangerList = DangerData.dangerList;
    for (int &opponentID : dangerList) {
        if (defencePairs.find(opponentID) == defencePairs.end()) {
            if (! doesRobotHaveBall(static_cast<unsigned int>(opponentID), false)) {
                return opponentID;
            }
        }
        else if (defencePairs[opponentID] == selfID) {
            return opponentID;
        }
    }

    return - 1;
}

}
}
}
