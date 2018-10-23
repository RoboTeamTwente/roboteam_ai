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

namespace rtt {
namespace ai {

class RobotDealer {
public:

    static std::vector<int> getClaimedRobots();

    static void setKeeper(int id);

    static int getKeeper();

    static bool getKeeperAvailable();

    static bool claimRobot(int id);

    static bool claimRobotForTactic(int id, std::string const &playName);

    static bool claimRobotForTactic(std::vector<int> ids, std::string const &playName);

    static std::map<std::string, std::set<int>> const &getRobotOwnerList();

    static bool releaseRobot(int id);

    static bool claimRobots(std::vector<int> ids);

    static bool releaseRobots(std::vector<int> ids);

private:

    FRIEND_TEST(RobotDealerTest, RobotDealerTest);

    static std::set<int> takenRobots;

    static std::map<std::string, std::set<int>> robotOwners;

    static std::atomic<int> keeper;

    static std::atomic<bool> isKeeperAvailable;

    static std::mutex robotOwnersLock;

    static std::mutex takenRobotsLock;

    static void printRobotDistribution();

    friend class HaltTactic;

    static void haltOverride();

    static void removeRobotFromOwnerList(int id);

    static bool checkLegalID(int ID);

    static bool isRobotFree(int ID);

};

} // ai
} // rtt

#endif //ROBOTEAM_AI_ROBOTDEALER_HPP
