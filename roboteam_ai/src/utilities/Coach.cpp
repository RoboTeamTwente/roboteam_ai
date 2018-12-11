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
    for (auto r : tacticMates) {
        if (r != selfID) {
            if (control::ControlUtils::hasClearVision(selfID, r, World::get_world(), 2)) {
                return r;
            }
        }
    }
    return - 1;
}
int Coach::pickDefensivePassTarget(int selfID) {

    auto world = World::get_world();
    auto us = world.us;
    int safelyness = 3;
    while (safelyness >= 0) {
        for (auto friendly : us) {
            if (control::ControlUtils::hasClearVision(selfID, friendly.id, world, safelyness)) {
                return friendly.id;
            }
        }
        safelyness --;
    }
    return - 1;
}

int Coach::pickOpponentToCover(int selfID) {
    dangerfinder::DangerData DangerData = dangerfinder::DangerFinder::instance().getMostRecentData();
    std::vector<int> dangerList = DangerData.dangerList;
    for(int & opponentID : dangerList) {
        if(defencePairs.find(opponentID) == defencePairs.end()) {
            if(!doesRobotHaveBall(opponentID, false)) {
                defencePairs.insert({opponentID, selfID});
                return opponentID;
            }
        } else if (defencePairs[opponentID] == selfID) {
            return opponentID;
        }
    }

    return -1;
}

int Coach::whichRobotHasBall(bool isOurTeam) {
    roboteam_msgs::World world = World::get_world();
    std::vector<roboteam_msgs::WorldRobot> robots;
    if (isOurTeam) {
        robots = world.us;
    } else {
        robots = world.them;
    }

    for (auto& robot:robots) {
        if (doesRobotHaveBall(robot.id, isOurTeam)) {
            return robot.id;
        }
    }
    return -1;
}

int Coach::doesRobotHaveBall(unsigned int robotID, bool isOurTeam) {
    auto robot = World::getRobotForId(robotID, isOurTeam);
    Vector2 ballPos = World::get_world().ball.pos;

    Vector2 deltaPos = (ballPos - robot->pos);
    double dist = deltaPos.length();
    double angle = deltaPos.angle();
    return ( (dist < 0.15) && (fabs(angle - robot->angle) < 0.4) );
}

Vector2 Coach::calculateBestPosition(int selfID) {
    int opponentID1 = Coach::whichRobotHasBall(false);
    if (opponentID1 == -1) {
        return Vector2 {0, -5};
    }
    auto opponentID2 = Coach::pickOpponentToCover(selfID);
    if (opponentID2 == -1) {
        return Vector2 {0, -5};
    }

    std::shared_ptr<roboteam_msgs::WorldRobot> robot1 = World::getRobotForId(static_cast<unsigned int>(opponentID1), false);
    std::shared_ptr<roboteam_msgs::WorldRobot> robot2 = World::getRobotForId(static_cast<unsigned int>(opponentID2), false);

    float robotAngle1 = robot1.get()->angle;
    float robotAngle2 = robot2.get()->angle;

    float angleBetweenRobots = atan((robot2->pos.y - robot1->pos.y) / (robot2->pos.x - robot1->pos.x));

    float angle1 = (angleBetweenRobots - robotAngle1) / 2;
    double angle2 = (M_PI + robotAngle2 - angleBetweenRobots) / 2;

    double distanceBetweenRobots = sqrt(pow(robot1->pos.x - robot2->pos.x, 2) + pow(robot2->pos.y - robot1->pos.y, 2));
    double length = distanceBetweenRobots * sin(angle2) / sin(M_PI - angle1 - angle2);

    double xLength = length * cos(angle1);
    double yLength = length * sin(angle1);

    Vector2 newPosition = {robot1->pos.x + xLength, robot1->pos.y + yLength};
    return newPosition;
}

}
}
}
