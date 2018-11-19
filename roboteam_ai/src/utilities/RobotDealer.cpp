//
// Created by baris on 16/11/18.
//
#include "RobotDealer.h"

namespace RobotDealer {

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
    for (auto &entry : robotOwners) {
        // Get the set
        entry.second.insert({ID, roleName});
    }

}
/// For internal use
/// Look at the world and see if there are more robots than on the map and if so put them as free
void RobotDealer::updateFromWorld() {

    std::lock_guard<std::mutex> lock(robotOwnersLock);

    auto worldUs = rtt::ai::World::get_world().us;
    std::set<int> robots;
    for (auto robot : worldUs) {
        robots.insert(robot.id);
    }
    std::set<int> currentRobots = RobotDealer::getRobots();
    for (auto robot : robots) {
        if (currentRobots.find(robot) == currentRobots.end()) {
            RobotDealer::addRobotToOwnerList(robot, "free", "free");
        }
    }

}

int RobotDealer::claimRobotForTactic(RobotDealer::ROBOT_TYPE feature, std::string roleName, std::string tacticName) {

    std::lock_guard<std::mutex> lock(robotOwnersLock);

    switch (feature) {
        // TODO add more things and the actual logic
    case CLOSEST_TO_BALL:return 0;

    case FAR_FROM_BALL:return 1;

    case READY_TO_DEFEND:return 1;

    case RANDOM: return 1;

    default:return 3;
    }

}
std::set<int> RobotDealer::getRobots() {

    std::lock_guard<std::mutex> lock(robotOwnersLock);

    std::set<int> res;

    for (auto tactic : robotOwners) {
        auto set = tactic.second;
        for (auto pair : set) {
            res.insert(pair.first);
        }
    }
    return res;
}
std::set<int> RobotDealer::getAvailableRobots() {

    std::lock_guard<std::mutex> lock(robotOwnersLock);

    auto set = robotOwners["free"];
    std::set<int> res;
    for (auto pair : set) {
        res.insert(pair.first);
    }
    return res;
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
std::vector<int> RobotDealer::findRobotsForTactic(std::string tacticName) {

    std::lock_guard<std::mutex> lock(robotOwnersLock);

    std::vector<int> res;
    for (auto tactic : robotOwners) {
        if (tactic.first == tacticName) {
            for (auto pair : tactic.second) {
                res.push_back(pair.first);
            }
        }
    }
    return res;
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
} // RobotDealer
