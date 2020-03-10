#include <roboteam_utils/Print.h>
#include "utilities/RobotDealer.h"
#include "coach/BallplacementCoach.h"
#include "coach/PassCoach.h"
#include "control/ControlUtils.h"
#include "treeinterp/BTFactory.h"
#include "world/Ball.h"
#include "world/World.h"

namespace rtt::ai::robotDealer {

std::map<std::string, std::set<std::pair<int, std::string>>> RobotDealer::robotOwners = {};

int RobotDealer::keeperID = -1;
bool RobotDealer::hasClaimedKeeper = false;
std::mutex RobotDealer::robotOwnersLock;

/// For internal use
/// Removes a robot with an ID from the map and if the tactic then is empty it removes the tactic
void RobotDealer::removeRobotFromOwnerList(int ID) {
    // For each tactic
    for (auto &tactic : robotOwners) {
        // Set of robots
        for (auto &robotPair : tactic.second) {
            if (robotPair.first == ID) {
                tactic.second.erase(robotPair);
                // If there are no more robots in the tactic
                if (tactic.second.empty()) {
                    std::string tacticToRemove = tactic.first;
                    robotOwners.erase(tacticToRemove);
                }
                addRobotToOwnerList(ID, "free", "free");
                return;  // TODO: test this function because it did not work before
            }
        }
    }
}

/// For internal use
/// Adds a robot to the map with a role and tactic
void RobotDealer::addRobotToOwnerList(int ID, const std::string &roleName, const std::string &tacticName) {
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
    auto worldUs = world::world->getUs();
    std::set<int> robotIDs;
    for (const auto &robot : worldUs) {
        robotIDs.insert(robot->id);
    }
    std::set<int> currentRobots = getRobots();
    for (const auto &robotID : robotIDs) {
        if (currentRobots.find(robotID) == currentRobots.end()) {
            if (robotID == keeperID) {
                RTT_ERROR("The keeper just got registered as a free robot this should never happen");
                continue;
            }
            std::lock_guard<std::mutex> lock(robotOwnersLock);
            addRobotToOwnerList(robotID, "free", "free");
        }
    }
}

int RobotDealer::claimRobotForTactic(const Field &field, RobotType feature, const std::string &roleName, const std::string &tacticName) {
    std::set<int> ids = getAvailableRobots();

    // convert the set to a vector here
    std::vector<int> idVector;
    idVector.assign(ids.begin(), ids.end());

    int id;
    if (!ids.empty()) {
        switch (feature) {
            default:
                RTT_WARNING("[Robotdealer] could not find a match for this feature ");
                return -1;

            case CLOSE_TO_BALL: {
                auto ball = world::world->getBall();
                auto robot = world::world->getRobotClosestToPoint(ball->getPos(), idVector, true);
                if (robot) {
                    id = robot->id;
                } else {
                    id = -1;
                }
                break;
            }

            case BETWEEN_BALL_AND_OUR_GOAL: {
                auto ball = world::world->getBall();
                rtt::Vector2 ourGoal = field.getOurGoalCenter();
                auto robots = world::world->getRobotsForIds(idVector, true);
                if (!robots.empty()) {
                    id = control::ControlUtils::getRobotClosestToLine(robots, ball->getPos(), ourGoal, true)->id;
                } else {
                    id = -1;
                }
                break;
            }
            case CLOSE_TO_OUR_GOAL: {
                rtt::Vector2 ourGoal = field.getOurGoalCenter();
                auto robot = world::world->getRobotClosestToPoint(ourGoal, idVector, true);
                if (robot) {
                    id = robot->id;
                } else {
                    id = -1;
                }
                break;
            }

            case CLOSE_TO_THEIR_GOAL: {
                rtt::Vector2 theirGoal = field.getTheirGoalCenter();
                auto robot = world::world->getRobotClosestToPoint(theirGoal, idVector, true);
                if (robot) {
                    id = robot->id;
                } else {
                    id = -1;
                }
                break;
            }

            case RANDOM: {
                id = *ids.begin();
                break;
            }

            case BALL_PLACEMENT_RECEIVER: {
                auto robot = world::world->getRobotClosestToPoint(rtt::ai::coach::g_ballPlacement.getBallPlacementPos(), idVector, true);
                if (robot) {
                    id = robot->id;
                } else {
                    id = -1;
                }
                // force the pass coach to use this receiver
                rtt::ai::coach::g_pass.resetPass(-1);
                rtt::ai::coach::g_pass.setRobotBeingPassedTo(id);

                break;
            }
            case WORKING_GENEVA: {
                int test = -1;
                for (auto r : ids) {
                    auto robot = rtt::ai::world::world->getRobotForId(r, true);
                    if (robot && robot->hasWorkingGeneva()) {
                        test = r;
                        break;
                    }
                }
                if (test == -1) {
                    id = *ids.begin();
                    break;
                }
                id = test;
                break;
            }
            case WORKING_BALL_SENSOR: {
                int test = -1;
                for (auto r : ids) {
                    auto robot = rtt::ai::world::world->getRobotForId(r, true);
                    if (robot && robot->hasWorkingBallSensor()) {
                        test = r;
                        break;
                    }
                }
                if (test == -1) {
                    id = *ids.begin();
                    break;
                }
                id = test;
                break;
            }
            case WORKING_GENEVA_BALLSENSOR: {
                int test = -1;
                for (auto r : ids) {
                    auto robot = rtt::ai::world::world->getRobotForId(r, true);
                    if (robot && robot->hasWorkingGeneva() && robot->hasWorkingBallSensor()) {
                        test = r;
                        break;
                    }
                }
                if (test == -1) {
                    id = *ids.begin();
                    break;
                }
                id = test;
                break;
            }
            case WORKING_DRIBBLER: {
                int test = -1;
                for (auto r : ids) {
                    auto robot = rtt::ai::world::world->getRobotForId(r, true);
                    if (robot && robot->hasWorkingDribbler()) {
                        test = r;
                        break;
                    }
                }
                if (test == -1) {
                    id = *ids.begin();
                    break;
                }
                id = test;
                break;
            }
        }
        std::lock_guard<std::mutex> lock(robotOwnersLock);
        unFreeRobot(id);
        addRobotToOwnerList(id, roleName, std::move(tacticName));
        return id;
    }
    // If a tactics gets here, it is probably because it was told it needs one more robot but the other tactic that needs
    // one less robot hasn't ben ticked yet so it did not disclaim its extra robot. It will be fine the next tick there is
    // no way around this
    return -1;
}

std::set<int> RobotDealer::getRobots() {
    std::lock_guard<std::mutex> lock(robotOwnersLock);

    std::set<int> ids;

    for (const auto &tactic : robotOwners) {
        auto set = tactic.second;
        for (const auto &pair : set) {
            ids.insert(pair.first);
        }
    }
    return ids;
}

std::set<int> RobotDealer::getAvailableRobots() {
    updateFromWorld();

    std::lock_guard<std::mutex> lock(robotOwnersLock);

    auto set = robotOwners["free"];
    std::set<int> ids;
    for (const auto &pair : set) {
        ids.insert(pair.first);
    }
    return ids;
}

std::map<std::string, std::set<std::pair<int, std::string>>> RobotDealer::getClaimedRobots() {
    std::lock_guard<std::mutex> lock(robotOwnersLock);
    return robotOwners;
}

void RobotDealer::releaseRobotForRole(const std::string &roleName) {
    std::lock_guard<std::mutex> lock(robotOwnersLock);

    auto test = robotOwners;

    // Find the ID
    /// Do NOT alt-enter->(const auto &) this function !!
    for (auto tactic : robotOwners) {
        auto set = tactic.second;
        for (auto pair : set) {
            if (pair.second == roleName) {
                removeRobotFromOwnerList(pair.first);
                return;
            }
        }
    }
    RTT_WARNING("Cannot release the robot. It does not exist in the robotOwners");
}

void RobotDealer::removeTactic(const std::string &tacticName) {
    std::lock_guard<std::mutex> lock(robotOwnersLock);
    /// Do NOT alt-enter->(const auto &) this function !!
    for (auto tactic : robotOwners) {
        if (tactic.first == tacticName) {
            for (auto robotPair : tactic.second) {
                removeRobotFromOwnerList(robotPair.first);
            }
            robotOwners.erase(tacticName);
            return;
        }
    }
}

std::set<int> RobotDealer::findRobotsForTactic(const std::string &tacticName) {
    std::lock_guard<std::mutex> lock(robotOwnersLock);

    std::set<int> ids;
    for (const auto &tactic : robotOwners) {
        if (tactic.first == tacticName) {
            for (const auto &pair : tactic.second) {
                ids.insert(pair.first);
            }
        }
    }
    return ids;
}

//  TODO: might want to add a tactic name here for confusion
int RobotDealer::findRobotForRole(const std::string &roleName) {
    std::lock_guard<std::mutex> lock(robotOwnersLock);

    for (const auto &tactic : robotOwners) {
        auto set = tactic.second;
        for (const auto &pair : set) {
            if (pair.second == roleName) {
                return pair.first;
            }
        }
    }
//    RTT_WARNING("Cannot find a robot with that Role Name: ", roleName);
    return -1;
}

/// When robot be free this bad boy anti free
void RobotDealer::unFreeRobot(int ID) {
    if (robotOwners["free"].find({ID, "free"}) != robotOwners["free"].end()) {
        robotOwners["free"].erase({ID, "free"});
    } else {
        RTT_WARNING("Cannot un free an anti free robot. id: ", ID);
    }
}

// std::map<std::string, std::set<std::pair<int, std::string>>> RobotDealer::robotOwners;
// map (string, set(pair(int, string)))

std::string RobotDealer::getTacticNameForRole(const std::string &role) {
    std::lock_guard<std::mutex> lock(robotOwnersLock);

    for (const auto &tactic : robotOwners) {
        for (const auto &pair : tactic.second) {
            if (pair.second == role) {
                return tactic.first;
            }
        }
    }
    RTT_WARNING("No robot with that role: ", role);
    return "";
}

std::string RobotDealer::getTacticNameForId(int ID) {
    std::lock_guard<std::mutex> lock(robotOwnersLock);

    for (const auto &tactic : robotOwners) {
        for (const auto &pair : tactic.second) {
            if (pair.first == ID) {
                return tactic.first;
            }
        }
    }
    RTT_WARNING("No robot with that ID: ", ID);
    return "";
}

std::string RobotDealer::getRoleNameForId(int ID) {
    std::lock_guard<std::mutex> lock(robotOwnersLock);

    for (const auto &tactic : robotOwners) {
        for (const auto &pair : tactic.second) {
            if (pair.first == ID) {
                return pair.second;
            }
        }
    }
    //RTT_WARNING("No robot with that ID: ", ID);
    return "";
}

void RobotDealer::halt() {
    {
        std::lock_guard<std::mutex> lock(robotOwnersLock);
        robotOwners.clear();
    }
    RobotDealer::updateFromWorld();
    hasClaimedKeeper = false;
}

/// set the keeper ID if its different than before
void RobotDealer::setKeeperID(int ID) {
    if (ID != keeperID) {
        keeperID = ID;
        hasClaimedKeeper = false;
        refresh();
    }
}

int RobotDealer::getKeeperID() {
    std::lock_guard<std::mutex> lock(robotOwnersLock);
    return keeperID;
}

void RobotDealer::claimKeeper() {
    if (!hasClaimedKeeper) {
        std::lock_guard<std::mutex> lock(robotOwnersLock);
        addRobotToOwnerList(keeperID, "Keeper", "Keeper");
        hasClaimedKeeper = true;
        RTT_INFO("The keeper is now robot ", keeperID)
    }
}

void RobotDealer::refresh() {
    halt();
    if (BTFactory::getCurrentTree() != "NaN" && BTFactory::getTree(BTFactory::getCurrentTree())) {
        BTFactory::getTree(BTFactory::getCurrentTree())->terminate(bt::Node::Status::Success);
    }
    if (keeperExistsInWorld()) claimKeeper();
}

bool RobotDealer::keeperExistsInWorld() {
    for (auto const &robot : world::world->getUs()) {
        if (robot && robot->id == getKeeperID()) {
            return true;
        }
    }
    return false;
}
bool RobotDealer::hasFree() {
    std::lock_guard<std::mutex> lock(robotOwnersLock);

    for (const auto &tactic : robotOwners) {
        auto set = tactic.second;
        for (const auto &pair : set) {
            if (pair.second == "free") {
                RTT_WARNING("There is a free robot with the ID: ", pair.first);
                return true;
            }
        }
    }
    return false;
}

}  // namespace rtt::ai::robotDealer
