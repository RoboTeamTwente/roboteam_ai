//
// Created by baris on 16/11/18.
//

#ifndef ROBOTEAM_AI_ROBOTDEALER_H
#define ROBOTEAM_AI_ROBOTDEALER_H

#include <map>
#include <set>
#include <mutex>
#include <vector>
#include "roboteam_utils/Vector2.h"

namespace robotDealer {

enum RobotType : short {
    CLOSE_TO_BALL,
    FAR_FROM_BALL,
    CLOSE_TO_OUR_GOAL,
    BETWEEN_BALL_AND_OUR_GOAL,
    CLOSE_TO_THEIR_GOAL,
    BALL_PLACEMENT_RECEIVER,
    RANDOM
};

class RobotDealer {

private:
    static bool useSeparateKeeper;
    static bool hasClaimedKeeper;
    static std::map<std::string, std::set<std::pair<int, std::string>>> robotOwners;
    static int keeperID;
    static std::mutex robotOwnersLock;
    static void removeRobotFromOwnerList(int ID);
    static void addRobotToOwnerList(int ID, std::string tacticName, std::string roleName);
    static void updateFromWorld();
    static std::set<int> getRobots();
    static int getRobotClosestToPoint(std::set<int> &ids, rtt::Vector2 position);
    static void unFreeRobot(int ID);
    static int getRobotClosestToLine(std::set<int> &ids, rtt::Vector2 point1, rtt::Vector2 point2,
            bool inBetweenPoints);
    static void claimKeeper();

public:
    static int claimRobotForTactic(RobotType feature, std::string tacticName, std::string roleName);
    static std::set<int> getAvailableRobots();
    static std::map<std::string, std::set<std::pair<int, std::string>>> getClaimedRobots();
    static void releaseRobotForRole(std::string roleName);
    static void removeTactic(std::string tacticName);
    static std::set<int> findRobotsForTactic(std::string tacticName);
    static int findRobotForRole(std::string roleName);
    static std::string getTacticNameForId(int ID);
    static std::string getRoleNameForId(int ID);
    static std::string getTacticNameForRole(std::string role);
    static void halt();
    static void setKeeperID(int ID);
    static int getKeeperID();
    static void refresh();

    static bool usesSeparateKeeper();
    static void setUseSeparateKeeper(bool useSeparateKeeper);

};
}
#endif //ROBOTEAM_AI_ROBOTDEALER_H
