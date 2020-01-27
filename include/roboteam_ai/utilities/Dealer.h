#ifndef RTT_ROBOTEAM_AI_SRC_UTILITIES_DEALER_H_
#define RTT_ROBOTEAM_AI_SRC_UTILITIES_DEALER_H_

/*
 * The purpose of Dealer is to go from pairs of (name -> dealerFlags) to pairs of (name -> robot id).
 * In other words, there is a requirement for each role in the tree, and Dealer maps the robot that fits those
 * requirements best to that role in the tree.
 */

#include <vector>
#include <unordered_map>
#include <iostream>
#include "gtest/gtest_prod.h"
#include <functional>

namespace rtt::ai {


/*
 * Set up a struct for dealerflags.
 *
 */
enum class DealerFlagTitle {
  CLOSE_TO_THEIR_GOAL,
  CLOSE_TO_OUR_GOAL,
  CLOSE_TO_BALL,
  HAS_WORKING_BALL_SENSOR,
  ROBOT_TYPE_50W,
  ROBOT_TYPE_30W,
};

class Dealer {
  FRIEND_TEST(DealerTest, it_claims);

  struct DealerFlag {
    DealerFlagTitle title;
    bool important;
    explicit DealerFlag(DealerFlagTitle title, bool important);
  };

  struct Role {
    std::string name;
    int robotId;
    explicit Role(std::string roleName, int robotId);
  };

 private:
  std::unordered_map<std::string, Role> claimedRoles;

 public:
  Dealer() = default;

  // Claim a robot. Returns Nothing if robot could not be claimed
  std::optional<int> claimRobot(std::vector<int> allRobots, const std::string& roleName, std::vector<DealerFlag> flags, bool allowShuffle);

  // Free robot
  void freeRobot(const std::string& roleName);

  // Get a list of claimed robots for a key
  std::vector<Role> getClaimedRoles();

  std::vector<int> getClaimedRobotIds();

  // Free all robots
  void freeAllRobots();

  // Reshuffle all robots to maximize flag utility
  void reshuffle();

  // get the optimal robot for a given set of flags
  std::optional<int> getOptimalRobot(std::vector<int> allRobots, const std::unordered_map<std::string, std::vector<DealerFlag>>& flagMap);

  std::vector<int> getFreeRobots(std::vector<int> allRobots, std::vector<int> claimedRobots);

  int getScoreForFlag(int robotId, DealerFlag flag);
};
}
#endif //RTT_ROBOTEAM_AI_SRC_UTILITIES_DEALER_H_
