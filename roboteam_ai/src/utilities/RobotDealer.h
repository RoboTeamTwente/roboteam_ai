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

namespace rtt {
namespace ai {

class RobotDealer {
    public:

        static std::set<int> getClaimedRobots();

        static std::set<int> getAvailableRobots();

        static bool claimKeeper(int id);

        static int getKeeper();

        static bool getKeeperAvailable();

        static bool claimRobot(int id);

        static int claimRandomRobot();

        static int claimRobotClosestToBall();

        static bool claimRobotForTactic(int id, std::string const &playName, std::string const &roleName);

        static bool claimRobotForTactic(std::set<int> ids, std::string const &playName, std::string const &roleName);

        static std::map<std::string, std::set<std::pair<int, std::string>>> const &getRobotOwnerList();

        static bool releaseKeeper();

        static bool releaseRobot(int id);

        static bool claimRobot(std::set<int> ids);

        static bool releaseRobot(std::set<int> ids);

        static void haltOverride();

    private:

        FRIEND_TEST(RobotDealerTest, RobotDealerTest);

        static std::set<int> takenRobots;

        static void emptyTakenRobots();

        static void emptyRobotOwners();

        static std::map<std::string, std::set<std::pair<int, std::string>>> robotOwners;

        static std::atomic<int> keeper;

        static std::atomic<bool> isKeeperAvailable;

        static std::mutex robotOwnersLock;

        static std::mutex takenRobotsLock;

        friend class HaltTactic;

        static void removeRobotFromOwnerList(int id);

        static bool validateID(int id);

        static bool isRobotAvailable(int id);

};

} // ai
} // rtt

#endif //ROBOTEAM_AI_ROBOTDEALER_HPP
