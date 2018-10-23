//
// Created by baris on 23/10/18.
//

#include "RobotDealer.h"

#define ROS_LOG_NAME "RobotDealer"

namespace rtt {
namespace ai {

std::set<int> RobotDealer::takenRobots;
// A map from tactic to robots it owns
std::map<std::string, std::set<int>> RobotDealer::robotOwners;

// Keeper and it's availability
// They are atomic for future compatibility
std::atomic<int> RobotDealer::keeper;
std::atomic<bool> RobotDealer::isKeeperAvailable(true);

// The locks
std::mutex RobotDealer::robotOwnersLock;
std::mutex RobotDealer::takenRobotsLock;

/// Returns a list of claimed robots
std::vector<int> RobotDealer::getClaimedRobots() {
    return std::vector<int>(takenRobots.begin(), takenRobots.end());
}

/// Set the keeper to an ID
void RobotDealer::setKeeper(int id) {
    keeper = id;
}

/// Returns the ID of the keeper
int RobotDealer::getKeeper() {
    return keeper;
}

/// Checks if the keeper is available (taken)
bool RobotDealer::getKeeperAvailable() {
    return isKeeperAvailable;
}

/// Makes a robot not available
bool RobotDealer::claimRobot(int id) {

    if (! RobotDealer::validateID(id)) {
        return false;
    }

    // Lock the robot
    std::lock_guard<std::mutex> ownerLock(robotOwnersLock);

    ROS_DEBUG_NAMED(ROS_LOG_NAME, "claimRobot with id: %i", id);

    // If we want the keeper
    if (id == keeper) {
        if (! isKeeperAvailable) {
            ROS_ERROR("Keeper already taken!");
            return false;
        }
        isKeeperAvailable = false;
        return true;
    }
    if (RobotDealer::isRobotFree(id)) {
        std::lock_guard<std::mutex> takenLock(takenRobotsLock);
        takenRobots.insert(id);
        return true;
    }
    else {
        ROS_ERROR("Robot %d is already claimed!", id);
        return false;
    }
}

/// Claims one robot for a tactic
bool RobotDealer::claimRobotForTactic(int id, std::string const &playName) {

    if (! RobotDealer::validateID(id)) {
        return false;
    }

    bool success = claimRobot(id);

    if (success) {
        std::lock_guard<std::mutex> lock(robotOwnersLock);
        robotOwners[playName].insert(id);
    }

    return success;
}

/// Claims multiple robots at once for a tactic
bool RobotDealer::claimRobotForTactic(std::vector<int> ids, std::string const &playName) {
    bool allClaimed = true;
    for (auto const id : ids) {
        // Check if all of the assignments are successful
        allClaimed &= claimRobotForTactic(id, playName);
    }
    return allClaimed;
}

/// Returns the map of the robots owned and the tactics that own them
std::map<std::string, std::set<int>> const &RobotDealer::getRobotOwnerList() {
    std::lock_guard<std::mutex> lock(robotOwnersLock);
    return robotOwners;
}

/// Releases a robot from being used
bool RobotDealer::releaseRobot(int id) {

    if (! RobotDealer::validateID(id)) {
        return false;
    }

    ROS_DEBUG_NAMED(ROS_LOG_NAME, "Releasing robot %i", id);

    if (id == keeper) {
        if (isKeeperAvailable) {
            ROS_ERROR("Goalkeeper was not claimed!");
            return false;
        }
        removeRobotFromOwnerList(id);
        isKeeperAvailable = true;
        return true;
    }

    if (RobotDealer::isRobotFree(id)) {
        ROS_ERROR_NAMED(ROS_LOG_NAME, "Tried to release an unclaimed robot: %d!", id);
        return false;
    }

    std::lock_guard<std::mutex> takenLock(takenRobotsLock);
    takenRobots.erase(id);

    removeRobotFromOwnerList(id);
    return true;
}

/// Claim multiple robots
bool RobotDealer::claimRobots(std::vector<int> ids) {
    bool allClaimed = true;
    for (int id : ids) {
        allClaimed &= claimRobot(id);
    }
    return allClaimed;
}

/// Releases multiple robots
bool RobotDealer::releaseRobots(std::vector<int> ids) {
    bool allReleased = true;
    for (int id : ids) {
        allReleased &= releaseRobot(id);
    }
    return allReleased;
}

/// Removes a robot from the robots owners list
void RobotDealer::removeRobotFromOwnerList(int id) {

    if (! RobotDealer::validateID(id)) {
        return;
    }

    std::lock_guard<std::mutex> lock(robotOwnersLock);
    boost::optional<std::string> tacticToRemove;

    // For each robot set list...
    for (auto &entry : robotOwners) {
        // Get the set
        auto &robotSet = entry.second;
        // Check if the robot is in there
        auto robotIt = robotSet.find(id);
        if (robotIt != robotSet.end()) {
            // If so, erase it
            robotSet.erase(robotIt);

            // And if the set is then empty, mark it for removal from the map
            if (robotSet.empty()) {
                tacticToRemove = entry.first;
            }

            break;
        }
    }
    // If there was a set empty after removal, remove it from the map
    if (tacticToRemove) {
        robotOwners.erase(*tacticToRemove);
    }
}

void RobotDealer::printRobotDistribution() {
    std::lock_guard<std::mutex> lock(robotOwnersLock);
    std::cout << "[RobotDistribution]\n";
    for (auto const &entry : robotOwners) {
        std::cout << entry.first << ":\n";
        for (auto const &id : entry.second) {
            std::cout << "\t- " << id << "\n";
        }
    }
}

/// Make all of the robots free in case of HALT
void RobotDealer::haltOverride() {
    std::lock_guard<std::mutex> lock(robotOwnersLock);
    ROS_WARN("Overriding claims for all robots because of HALT");
    takenRobots.clear();
}

/// Checks if a robot ID is legal
bool RobotDealer::validateID(int ID) {
    if (ID < 0 || ID > 10000) {
        ROS_ERROR("Illegal robot ID");
        return false;
    }
    return true;
}

/// Checks if a robot is free
bool RobotDealer::isRobotFree(int ID) {
    return ! (takenRobots.find(ID) != takenRobots.end());
}

} // ai
} // rtt

#undef ROS_LOG_NAME