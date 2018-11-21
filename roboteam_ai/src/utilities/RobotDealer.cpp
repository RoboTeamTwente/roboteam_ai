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
                return; // TODO: test this function because it did not work before
            }
        }
    }


}

/// For internal use
/// Adds a robot to the map with a role and tactic
void RobotDealer::addRobotToOwnerList(int ID, std::string roleName, std::string tacticName) {
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
    int id;
    if (! ids.empty()) {

        switch (feature) {

            default:
                return - 1;

            case closeToBall: {
                rtt::Vector2 ball = rtt::ai::World::getBall().pos;
                id = getRobotClosestToPoint(ids, ball);
                break;
            }

            case readyToDefend: {
                rtt::Vector2 ourGoal = rtt::ai::Field::get_our_goal_center();
                id = getRobotClosestToPoint(ids, ourGoal);
                break;
            }

            case readyToAttack: {
                rtt::Vector2 theirGoal = rtt::ai::Field::get_their_goal_center();
                id = getRobotClosestToPoint(ids, theirGoal);
                break;
            }

            case random: {
                id = *ids.begin();
                break;
            }

        }
        std::lock_guard<std::mutex> lock(robotOwnersLock);
        RobotDealer::unFreeRobot(id);
        RobotDealer::addRobotToOwnerList(id, std::move(tacticName), std::move(roleName));
        return id;
    }
    ROS_INFO_STREAM("Found no free robots in robot dealer");
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

int RobotDealer::getRobotClosestToPoint(std::set<int> &ids, rtt::Vector2 position) {
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

/// When robot be free this bad boy anti free
void RobotDealer::unFreeRobot(int ID) {

    if(robotOwners["free"].find({ID, "free"}) != robotOwners["free"].end()) {
        robotOwners["free"].erase({ID, "free"});
    } else {
        ROS_ERROR("Cannot un free an anti free robot");
    }

}

} // RobotDealer
