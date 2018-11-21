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
    int id = - 1;

    if (! ids.empty()) {

        switch (feature) {
        default: return - 1;
            // TODO add more cases
        case closeToBall: {

            rtt::Vector2 ball = rtt::ai::World::getBall().pos;
            id = getRobotClosestToPoint(ids, ball);
            break;
        }

        case readyToDefend: {

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
    return - 1;
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
    if (! ids.empty()) {
        int closestID = - 1;
        double distance = 100000000.0;
        for (auto &id : ids) {
            rtt::Vector2 robot = rtt::ai::World::getRobotForId((unsigned int) id, true).get().pos;
            double dBallRobot = (robot - position).length();
            if (dBallRobot < distance) {
                closestID = id;
                distance = dBallRobot;
            }
        }
        return closestID;
    }
    else return - 1;
}

} // RobotDealer
