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

// The lock
std::mutex RobotDealer::mutex;

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
    // Lock the robots
    std::lock_guard<std::mutex> lock(mutex);

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
    bool success = claimRobot(id);

    if (success) {
        std::lock_guard<std::mutex> lock(mutex);
        robotOwners[playName].insert(id);
    }

    return success;
}

bool RobotDealer::claimRobotsForTactic(std::vector<int> ids, std::string const &playName) {
    bool allClaimed = true;
    for (auto const id : ids) {
        // Check if all of the assignments are successful
        allClaimed &= claimRobotForTactic(id, playName);
    }
    return allClaimed;
}

std::map<std::string, std::set<int>> const &RobotDealer::getRobotOwnerList() {
    return robotOwners;
}

bool RobotDealer::releaseRobots(int id) {
    ROS_DEBUG_NAMED(ROS_LOG_NAME, "Releasing robot %i", id);

    removeRobotFromOwnerList(id);

    if (id == keeper) {
        if (isKeeperAvailable) {
            ROS_ERROR("Goalkeeper was not claimed!");
            return false;
        }

        isKeeperAvailable = true;
        return true;
    }

    if (takenRobots.find(id) == takenRobots.end()) {
        ROS_ERROR_NAMED(ROS_LOG_NAME, "Tried to release an unclaimed robot: %d!", id);
        return false;
    }

    // available_robots.insert(id);
    takenRobots.erase(id);
    return true;
}

bool RobotDealer::claimRobots(std::vector<int> ids) {
    bool allClaimed = true;
    for (int id : ids) {
        allClaimed &= claimRobot(id);
    }
    return allClaimed;
}

bool RobotDealer::releaseRobots(std::vector<int> ids) {
    bool allReleased = true;
    for (int id : ids) {
        allReleased &= releaseRobots(id);
    }
    return allReleased;
}

void RobotDealer::removeRobotFromOwnerList(int id) {

    std::lock_guard<std::mutex> lock(mutex);
    boost::optional<std::string> playToRemove;

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
                playToRemove = entry.first;
            }

            break;
        }
    }

    // If there was a set empty after removal, remove it from the map
    if (playToRemove) {
        robotOwners.erase(*playToRemove);
    }
}

void RobotDealer::printRobotDistribution() {
    std::lock_guard<std::mutex> lock(mutex);
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
    std::lock_guard<std::mutex> lock(mutex);
    ROS_WARN("Overriding claims for all robots because of HALT");
    takenRobots.clear();
}
bool RobotDealer::checkLegalID(int ID) {
    return true; // TODO see what is a legal ID and put ot here
}
bool RobotDealer::isRobotFree(int ID) {
    return ! (takenRobots.find(ID) != takenRobots.end());
}

} // ai
} // rtt

#undef ROS_LOG_NAME