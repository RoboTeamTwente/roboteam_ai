//
// Created by baris on 16/11/18.
//

#ifndef ROBOTEAM_AI_ROBOTDEALER_H
#define ROBOTEAM_AI_ROBOTDEALER_H

#include <utility>
#include <map>
#include <set>
#include <mutex>
#include <vector>
#include "roboteam_utils/Vector2.h"
#include "ros/ros.h"

namespace rtt {
namespace ai {
namespace robotDealer {

enum RobotType : short {
    CLOSE_TO_BALL,
    FAR_FROM_BALL,
    CLOSE_TO_OUR_GOAL,
    BETWEEN_BALL_AND_OUR_GOAL,
    CLOSE_TO_THEIR_GOAL,
    BALL_PLACEMENT_RECEIVER,
    RANDOM,
    WORKING_GENEVA,
    WORKING_BALL_SENSOR,
    WORKING_DRIBBLER
};

class RobotDealer {

private:
    static bool hasClaimedKeeper;
    static std::map<std::string, std::set<std::pair<int, std::string>>> robotOwners;
    static int keeperID;
    static std::mutex robotOwnersLock;
    static void removeRobotFromOwnerList(int ID);
    static void addRobotToOwnerList(int ID, const std::string& tacticName, const std::string& roleName);
    static void updateFromWorld();
    static std::set<int> getRobots();
    static void unFreeRobot(int ID);

    static void claimKeeper();

public:
    static int claimRobotForTactic(RobotType feature, const std::string& tacticName, const std::string& roleName);
    static std::set<int> getAvailableRobots();
    static std::map<std::string, std::set<std::pair<int, std::string>>> getClaimedRobots();
    static void releaseRobotForRole(const std::string& roleName);
    static void removeTactic(const std::string& tacticName);
    static std::set<int> findRobotsForTactic(const std::string& tacticName);
    static int findRobotForRole(const std::string& roleName);
    static std::string getTacticNameForId(int ID);
    static std::string getRoleNameForId(int ID);
    static std::string getTacticNameForRole(const std::string& role);
    static void halt();
    static void setKeeperID(int ID);
    static int getKeeperID();
    static void refresh();
    static bool keeperExistsInWorld();
    static bool hasFree();
    static bool robotBlockedByInterface(int id);

};

} //robotDealer
} //ai
} //rtt
#endif //ROBOTEAM_AI_ROBOTDEALER_H
