//
// Created by baris on 16/11/18.
//

#ifndef ROBOTEAM_AI_ROBOTDEALERR_H
#define ROBOTEAM_AI_ROBOTDEALERR_H

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

enum RobotType {
    closeToBall,
    farFromBall,
    closeToOurGoal,
    betweenBallAndOurGoal,
    closeToTheirGoal,
    random
};

class RobotDealer {

    private:

        std::map<std::string, std::set<std::pair<int, std::string>>> robotOwners;

        int keeperID;

        std::mutex robotOwnersLock;

        void removeRobotFromOwnerList(int ID);

        void addRobotToOwnerList(int ID, std::string tacticName, std::string roleName);

        void updateFromWorld();

        std::set<int> getRobots();

        int getRobotClosestToPoint(std::set<int> &ids, rtt::Vector2 position);

        void unFreeRobot(int ID);

        int getRobotClosestToLine(std::set<int> &ids, rtt::Vector2 point1, rtt::Vector2 point2,
         bool inBetweenPoints);

    public:

        int claimRobotForTactic(RobotType feature, std::string tacticName, std::string roleName);

        std::set<int> getAvailableRobots();

        std::map<std::string, std::set<std::pair<int, std::string>>> getClaimedRobots();

        void releaseRobotForRole(std::string roleName);

        void removeTactic(std::string tacticName);

        std::set<int> findRobotsForTactic(std::string tacticName);

        int findRobotForRole(std::string roleName);

        std::string getTacticNameForId(int ID);
        std::string getRoleNameForId(int ID);
        std::string getTacticNameForRole(std::string role);
        void halt();
        void setKeeperID(int ID);
        int getKeeperID();

};
RobotDealer* robotDealer;

} //robotDealer
} //ai
} //rtt
#endif //ROBOTEAM_AI_ROBOTDEALER_H
