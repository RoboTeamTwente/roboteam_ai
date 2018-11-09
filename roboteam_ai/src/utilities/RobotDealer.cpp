//
// Created by baris on 23/10/18.
//

#include "RobotDealer.h"

#define ROS_LOG_NAME "RobotDealer"

namespace rtt {
namespace ai {

std::set<int> RobotDealer::takenRobots;
// A map from tactic to robots it owns
std::map<std::string, std::set<std::pair<int, std::string>>> RobotDealer::robotOwners;

// Keeper and it's availability
// They are atomic for future compatibility
std::atomic<int> RobotDealer::keeper;
std::atomic<bool> RobotDealer::isKeeperAvailable(true);

// The locks
std::mutex RobotDealer::robotOwnersLock;
std::mutex RobotDealer::takenRobotsLock;

/// Returns a list of claimed robots
std::set<int> RobotDealer::getClaimedRobots() {
    std::set<int> ids = std::set<int>(takenRobots.begin(), takenRobots.end());
    if (! isKeeperAvailable) {
        ids.insert(keeper);
    }
    return ids;
}

std::set<int> RobotDealer::getAvailableRobots() {
    auto worldUs = World::get_world().us;
    std::set<int> takenIDs = getClaimedRobots();
    std::set<int> availableIDs;

    for (auto &robot : worldUs) {
        auto id = robot.id;
        if (takenIDs.find(id) == takenIDs.end()) {
            availableIDs.insert(id);
        }
    }
    return availableIDs;
}

/// Set the keeper to an ID
bool RobotDealer::claimKeeper(int id) {
    if (isKeeperAvailable) {
        keeper = id;
        isKeeperAvailable = false;
        return true;
    }
    else {
        ROS_ERROR("Keeper already taken!");
        return false;
    }
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
        return claimKeeper(id);
    }

    if (RobotDealer::isRobotAvailable(id)) {
        std::lock_guard<std::mutex> takenLock(takenRobotsLock);
        takenRobots.insert(id);
        return true;
    }
    else {
        ROS_ERROR("Robot %d is already claimed!", id);
        return false;
    }
}

int RobotDealer::claimRandomRobot() {

    std::set<int> availableIDs = getAvailableRobots();
    if (! availableIDs.empty()) {
        int randomID = *availableIDs.begin();
        int id = claimRobot(randomID);
        return id;
    }
    else return - 1;
}

int RobotDealer::claimRobotClosestToBall() {
    auto worldBall = World::getBall();
    Vector2 ball = worldBall.pos;
    auto worldUs = World::get_world().us;
    double minDistanceSquared = 1000000000.1;
    double distanceSquared, dx, dy;
    int id = - 1;
    for (auto &robot : worldUs) {
        dx = (robot.pos.x - ball.x);
        dy = (robot.pos.y - ball.y);
        distanceSquared = dx*dx + dy*dy;
        if (distanceSquared < minDistanceSquared) {
            minDistanceSquared = distanceSquared;
            id = robot.id;
        }
    }
    if (id != -1) {
        claimRobot(id);
    }
    return id;
}

int RobotDealer::claimRobotClosestToPoint(Vector2 pos) {
    //TODO: make this.
    return - 1;
}

/// Claims one robot for a tactic
bool RobotDealer::claimRobotForTactic(std::pair<int, std::string> const &idNamePair, std::string const &tacticName) {

    auto id = idNamePair.first;
    auto roleName = idNamePair.second;

    bool successClaimed = claimRobot(id);
    if (successClaimed) {
        std::lock_guard<std::mutex> lock(robotOwnersLock);
        std::pair<int, std::string> robotRole = {id, roleName};
        robotOwners[tacticName].insert(robotRole);
    }

    return successClaimed;
}

/// Claims multiple robots at once for a tactic
bool RobotDealer::claimRobotForTactic(std::set<std::pair<int, std::string>> const &roleSet,
        std::string const &tacticName) {

    bool allClaimed = true;
    for (auto const &idNamePair : roleSet) {
        //auto id = idNamePair.first;
        //auto roleName = idNamePair.second;
        allClaimed &= claimRobotForTactic(idNamePair, tacticName);
    }
    return allClaimed;
}

int RobotDealer::findRobotForRole(std::string const &roleName) {

    for (auto &tacticSet : robotOwners) {
        auto robotID = findRobotForRole(tacticSet.first, roleName);
        if (robotID != - 1) return robotID;
    }
    return - 1;
}

int RobotDealer::findRobotForRole(std::string const &tacticName, std::string const &roleName) {

    auto tacticSet = robotOwners[tacticName];
    // Check if the robot is in there
    for (auto &robotPair : tacticSet) {
        if (robotPair.second == roleName) {
            return robotPair.first;
        }
    }

    return - 1;
}

/// Returns the map of the robots owned and the tactics that own them
std::map<std::string, std::set<std::pair<int, std::string>>> const &RobotDealer::getRobotOwnerList() {
    std::lock_guard<std::mutex> lock(robotOwnersLock);
    return robotOwners;
}

bool RobotDealer::releaseKeeper() {
    if (isKeeperAvailable) {
        ROS_ERROR("Goalkeeper was not claimed!");
        return false;
    }
    keeper = - 1;
    removeRobotFromOwnerList(keeper);
    isKeeperAvailable = true;
    return true;

}

/// Releases a robot from being used
bool RobotDealer::releaseRobot(int id) {

    if (id == keeper) {
        ROS_DEBUG_NAMED(ROS_LOG_NAME, "Releasing keeper with id %i", id);
        return releaseKeeper();
    }

    ROS_DEBUG_NAMED(ROS_LOG_NAME, "Releasing robot %i", id);

    if (RobotDealer::isRobotAvailable(id)) {
        ROS_ERROR_NAMED(ROS_LOG_NAME, "Tried to release an unclaimed robot: %d!", id);
        return false;
    }
    std::lock_guard<std::mutex> takenLock(takenRobotsLock);
    takenRobots.erase(id);
    removeRobotFromOwnerList(id);
    return true;
}

/// Claim multiple robots
bool RobotDealer::claimRobot(std::set<int> ids) {
    bool allClaimed = true;
    for (int id : ids) {
        allClaimed &= claimRobot(id);
    }
    return allClaimed;
}

/// Releases multiple robots
bool RobotDealer::releaseRobot(std::set<int> ids) {
    bool allReleased = true;
    for (int id : ids) {
        allReleased &= releaseRobot(id);
    }
    return allReleased;
}

/// Removes a robot from the robots owners list
void RobotDealer::removeRobotFromOwnerList(int id) {

    std::lock_guard<std::mutex> lock(robotOwnersLock);
    boost::optional<std::string> tacticToRemove;

    // For each robot set list...

    for (auto &entry : robotOwners) {
        // Get the set
        auto &robotSet = entry.second;
        // Check if the robot is in there
        for (auto &robotPair : robotSet) {
            if (robotPair.first == id) {
                robotSet.erase(robotPair);

                if (robotSet.empty()) {
                    tacticToRemove = entry.first;
                    robotOwners.erase(*tacticToRemove);
                    return;
                }
            }
        }
    }
}

/// Make all of the robots free in case of HALT
void RobotDealer::haltOverride() {
    RobotDealer::emptyRobotOwners();
    RobotDealer::emptyTakenRobots();
}

/// Checks if a robot ID is legal
bool RobotDealer::validateID(int id) {
    if (World::getRobotForId(id, true)) return true;
    else return false;
}

/// Checks if a robot is free
bool RobotDealer::isRobotAvailable(int id) {
    return takenRobots.find(id) == takenRobots.end();
}

/// Clears the takenRobots
void RobotDealer::emptyTakenRobots() {
    std::lock_guard<std::mutex> lock(takenRobotsLock);
    takenRobots.clear();
}

/// Clears the robotOwners
void RobotDealer::emptyRobotOwners() {
    std::lock_guard<std::mutex> lock(robotOwnersLock);
    robotOwners.clear();
}

} // ai
} // rtt

#undef ROS_LOG_NAME