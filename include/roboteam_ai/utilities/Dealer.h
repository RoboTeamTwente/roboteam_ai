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
#include "include/roboteam_ai/world_new/views/RobotView.hpp"
#include "include/roboteam_ai/world_new/views/WorldDataView.hpp"

namespace rtt::ai {

namespace v = rtt::world_new::view;

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

enum class DealerFlagPriority {
  LOW_PRIORITY,
  MEDIUM_PRIORITY,
  HIGH_PRIORITY,
};

struct Data {
  v::WorldDataView world;
  world::Field * field; // TODO make this neater after field refactor
};

class Dealer {
  FRIEND_TEST(DealerTest, it_properly_distributes_robots);

 public:
  struct DealerFlag {
    DealerFlagTitle title;
    DealerFlagPriority priority;
    explicit DealerFlag(DealerFlagTitle title, DealerFlagPriority important);
  };

  using FlagMap = std::unordered_map<std::string, std::vector<DealerFlag>>;
  Dealer() = default;
  std::unordered_map<std::string, v::RobotView> distribute(const Data& data, std::vector<v::RobotView> allRobots, const FlagMap& flagMap);
  double getScoreForFlag(const Data& data, v::RobotView robot, DealerFlag flag);
  std::vector<std::vector<double>> getScoreMatrix(const Data& data, std::vector<v::RobotView> &allRobots, const FlagMap &flagMap);

  // This function is virtual such that it can be mocked in the tests.
  // the performance hit is minimal (in the scope of nanoseconds)
  virtual double getDefaultFlagScores(const Data& data, const v::RobotView &robot, const DealerFlag &flag) ;
  double getFactorForPriority(const DealerFlag &flag);
  virtual ~Dealer() = default;

};
}
#endif //RTT_ROBOTEAM_AI_SRC_UTILITIES_DEALER_H_
