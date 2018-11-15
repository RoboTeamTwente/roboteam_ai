//
// Created by baris on 23/10/18.
//

#ifndef ROBOTEAM_AI_ROBOTDEALER_HPP
#define ROBOTEAM_AI_ROBOTDEALER_HPP

#include <set>
#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <gtest/gtest_prod.h>
#include <boost/optional.hpp>
#include <mutex>
#include "World.h"
#include "Field.h"

namespace rtt {
namespace ai {

class RobotDealer {
    public:

        static std::set<int> getClaimedRobots();

        static std::set<int> getAvailableRobots();

        static bool isRobotAvailable(int id);

        static int findRobotForRole(std::string const &roleName);

        static int findRobotForRole(std::string const &roleName, std::string const &tacticName);

        static int claimRandomRobot();

        static int claimRobotClosestToBall();

        static int claimRobotClosestToPoint(Vector2 pos);

        static bool claimRobotForTactic(std::pair<int, std::string>const &idNamePair , std::string const &tacticName);

        static bool claimRobotForTactic(std::set<std::pair<int, std::string>>const &roleSet,
                std::string const &tacticName);

        static bool releaseRobot(int id);

        static bool releaseRobot(std::set<int> ids);

        static int getKeeperID();

        static bool claimKeeper(int id);

        static bool releaseKeeper();

        static bool isKeeperAvailable();

        static std::map<std::string, std::set<std::pair<int, std::string>>> const &getRobotOwnerList();

        static void haltOverride();

    private:

        FRIEND_TEST(RobotDealerTest, RobotDealerTest);


        static bool claimRobot(int id);

        static bool claimRobot(std::set<int> ids);

        static std::set<int> takenRobots;

        static void emptyTakenRobots();

        static void emptyRobotOwners();

        static std::map<std::string, std::set<std::pair<int, std::string>>> robotOwners;

        static std::atomic<int> keeper;

        static std::atomic<bool> keeperAvailable;

        static std::mutex robotOwnersLock;

        static std::mutex takenRobotsLock;

        static void removeRobotFromOwnerList(int id);

        static bool validateID(int id);

};

} // ai
} // rtt

#endif //ROBOTEAM_AI_ROBOTDEALER_HPP
