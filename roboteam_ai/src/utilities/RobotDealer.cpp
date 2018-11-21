#include <utility>

//
// Created by baris on 16/11/18.
//
#include "RobotDealer.h"

namespace robotDealer {

std::map<std::string, std::set<std::pair<int, std::string>>> RobotDealer::robotOwners;

std::mutex RobotDealer::robotOwnersLock;

/// For internal use
/// Removes a robot with an ID from the map and if the tactic then is empty it removes the tactic
void RobotDealer::removeRobotFromOwnerList(int ID) {
    // For each robot set list...
    for (auto &entry : robotOwners) {
        // Get the set
        auto robotSet = entry.second;
        // Check if the robot is in there
        for (auto &robotPair : robotSet) {
            if (robotPair.first == ID) {
                robotSet.erase(robotPair);
                // If there are no more robots in the tactic
                if (robotSet.empty()) {
                    std::string tacticToRemove = entry.first;
                    robotOwners.erase(tacticToRemove);
                }
                return;
            }
        }
    }
}

/// For internal use
/// Adds a robot to the map with a role and tactic
void RobotDealer::addRobotToOwnerList(int ID, std::string tacticName, std::string roleName) {
    // If tactic does not exist
    if (robotOwners.find(tacticName) == robotOwners.end()) {
        std::set<std::pair<int, std::string>> set = {{ID, roleName}};
        robotOwners[tacticName] = set;
        return;
    }

    // Seems tactic does exist
    robotOwners[tacticName].insert({ID, roleName});

}
/// For internal use
/// Look at the world and see if there are more robots than on the map and if so put them as free
void RobotDealer::updateFromWorld() {

    auto worldUs = rtt::ai::World::get_world().us;
    std::set<int> robots;
    for (auto robot : worldUs) {
        robots.insert(robot.id);
    }
    std::set<int> currentRobots = RobotDealer::getRobots();
    for (auto robot : robots) {
        if (currentRobots.find(robot) == currentRobots.end()) {
            std::lock_guard<std::mutex> lock(robotOwnersLock);
            RobotDealer::addRobotToOwnerList(robot, "free", "free");
        }
    }

}

int RobotDealer::claimRobotForTactic(RobotDealer::RobotType feature, std::string roleName, std::string tacticName) {

    std::set<int> ids = RobotDealer::getAvailableRobots();

    if (! ids.empty()) {
        int id;
        switch (feature) {
        default: return - 1;
            // TODO add more cases
        case closeToBall: {

            rtt::Vector2 ball = rtt::ai::World::getBall().pos;
            id = getRobotClosestToPoint(ids, ball);
            break;
        }

        case betweenBallAndOurGoal: {
            rtt::Vector2 ball = rtt::ai::World::getBall().pos;
            rtt::Vector2 ourGoal = rtt::ai::Field::get_our_goal_center();
            id = getRobotClosestToLine(ids, ball, ourGoal, true);
            break;
        }

        case random: {
            id = *ids.begin();
            break;
        }

        }
        std::lock_guard<std::mutex> lock(robotOwnersLock);
        RobotDealer::addRobotToOwnerList(id, std::move(tacticName), std::move(roleName));
        return id;
    }
    else return - 1;
}

std::set<int> RobotDealer::getRobots() {

    std::lock_guard<std::mutex> lock(robotOwnersLock);

    std::set<int> ids;

    for (auto tactic : robotOwners) {
        auto set = tactic.second;
        for (auto pair : set) {
            ids.insert(pair.first);
        }
    }
    return ids;
}
std::set<int> RobotDealer::getAvailableRobots() {

    RobotDealer::updateFromWorld();

    std::lock_guard<std::mutex> lock(robotOwnersLock);

    auto set = robotOwners["free"];
    std::set<int> ids;
    for (auto pair : set) {
        ids.insert(pair.first);
    }
    return ids;
}
std::map<std::string, std::set<std::pair<int, std::string>>> RobotDealer::getClaimedRobots() {
    return robotOwners;
}

void RobotDealer::releaseRobotForRole(std::string roleName) {

    std::lock_guard<std::mutex> lock(robotOwnersLock);

    // Find the ID
    for (auto tactic : robotOwners) {
        auto set = tactic.second;
        for (auto pair : set) {
            if (pair.second == roleName) {
                removeRobotFromOwnerList(pair.first);
                return;
            }
        }
    }
    std::cerr << "Cannot release the robot it does not exist in the robotOwners" << std::endl;

}
void RobotDealer::removeTactic(std::string tacticName) {

    std::lock_guard<std::mutex> lock(robotOwnersLock);

    for (auto tactic : robotOwners) {
        if (tactic.first == tacticName) {
            robotOwners.erase(tacticName);
            return;
        }
    }
    std::cerr << "Cannot remove tactic the tactic does not exist" << std::endl;
}
std::set<int> RobotDealer::findRobotsForTactic(std::string tacticName) {

    std::lock_guard<std::mutex> lock(robotOwnersLock);

    std::set<int> ids;
    for (auto tactic : robotOwners) {
        if (tactic.first == tacticName) {
            for (auto pair : tactic.second) {
                ids.insert(pair.first);
            }
        }
    }
    return ids;
}

int RobotDealer::findRobotForRole(std::string roleName) {

    std::lock_guard<std::mutex> lock(robotOwnersLock);

    for (auto tactic : robotOwners) {
        auto set = tactic.second;
        for (auto pair : set) {
            if (pair.second == roleName) {
                return pair.first;
            }
        }
    }
    std::cerr << "Cannot find a robot with that Role Name" << std::endl;
    return - 1;
}

int RobotDealer::getRobotClosestToPoint(std::set<int> &ids, rtt::Vector2 &position) {
    int closestID = - 1;
    double distance = 100000000.0;
    for (auto &id : ids) {
        rtt::Vector2 robotPos = rtt::ai::World::getRobotForId((unsigned int) id, true).get().pos;
        double dRobotToPoint = (robotPos - position).length();
        if (dRobotToPoint < distance) {
            closestID = id;
            distance = dRobotToPoint;
        }
    }
    return closestID;
}

int RobotDealer::getRobotClosestToLine(std::set<int> &ids, rtt::Vector2 &point1, rtt::Vector2 &point2,
        bool inBetweenPoints) {

    // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    int closestID = - 1;
    double distance = 100000000.0;
    for (auto &id : ids) {
        rtt::Vector2 robotPos = rtt::ai::World::getRobotForId((unsigned int) id, true).get().pos;
        double deltaY = point2.y - point1.y;
        double deltaX = point2.x - point1.x;
        double numerator = abs(deltaY*robotPos.x - deltaX*robotPos.y + point2.x*point1.y - point2.y*point1.x);
        double denominator = sqrt(deltaY*deltaY + deltaX*deltaX);
        double dRobotToLine = numerator/denominator;
        if (dRobotToLine > distance) continue;

        if (inBetweenPoints) {
            // if we want to check in between the points ...
            // for variables len**: R = robot, 1 = point 1, 2 = point 2
            // check if the angle is more than or less than 90 degrees by using pythagoras (in)equality
            // if it is more, change the distance dRobotToLine to the distance to point 1 or 2
            // instead of the distance to the line

            double len1R = ((rtt::Vector2) point1 - robotPos).length();
            double len2R = ((rtt::Vector2) point1 - robotPos).length();
            double len12 = ((rtt::Vector2) point1 - robotPos).length();
            if (len1R < len2R) {
                double pythagoras = len1R*len1R + len12*len12 - len2R*len2R;
                if (pythagoras < 0) dRobotToLine = len1R;
                if (dRobotToLine >= distance) continue;
            }
            else {
                double pythagoras = len2R*len2R + len12*len12 - len1R*len1R;
                if (pythagoras < 0) dRobotToLine = len2R;
                if (len2R >= distance) continue;
            }
        }

        closestID = id;
        distance = dRobotToLine;

    }
    return closestID;
}

} // RobotDealer
