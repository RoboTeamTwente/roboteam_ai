#ifndef RTT_ROBOTEAM_AI_SRC_UTILITIES_DEALER_H_
#define RTT_ROBOTEAM_AI_SRC_UTILITIES_DEALER_H_

/*
 * The purpose of Dealer is to go from pairs of (name -> dealerFlags) to pairs of (name -> robot id).
 * In other words, there is a requirement for each role in the tree, and Dealer maps the robot that fits those
 * requirements best to that role in the tree.
 */

#include <vector>
#include <map>
#include <iostream>
#include "gtest/gtest_prod.h"
#include "world_new/views/RobotView.hpp"
#include "world_new/views/WorldDataView.hpp"
#include "world/Field.h"

namespace rtt::ai {

namespace v = rtt::world_new::view;

// Set up a struct for dealerflags. Set up a struct for dealerflags.
enum class DealerFlagTitle {
  CLOSE_TO_THEIR_GOAL,
  CLOSE_TO_OUR_GOAL,
  CLOSE_TO_BALL,
  WITH_WORKING_BALL_SENSOR,
  WITH_WORKING_DRIBBLER,
  ROBOT_TYPE_50W,
  ROBOT_TYPE_30W,
  READY_TO_INTERCEPT_GOAL_SHOT,
  NOT_IMPORTANT,
  KEEPER
};

enum class DealerFlagPriority {
  LOW_PRIORITY,
  MEDIUM_PRIORITY,
  HIGH_PRIORITY,
  REQUIRED
};

class Dealer {
  FRIEND_TEST(DealerTest, it_properly_distributes_robots);
  FRIEND_TEST(DealerTest, the_score_factor_increases_with_priority);

 public:
  struct DealerFlag {
    DealerFlagTitle title;
    DealerFlagPriority priority;
    explicit DealerFlag(DealerFlagTitle title, DealerFlagPriority priority);
  };
  using FlagMap = std::map<std::string, std::vector<DealerFlag>>;
  Dealer(v::WorldDataView world, world::Field * field);
  virtual ~Dealer() = default; // needed for test
  // Create a distribution of robots
  std::unordered_map<std::string, v::RobotView> distribute(const std::vector<v::RobotView> &allRobots, const FlagMap &flagMap);

 protected:
  // This function is virtual such that it can be mocked in the tests.
  // the performance hit is minimal (in the scope of nanoseconds)
  virtual double getDefaultFlagScores(const v::RobotView &robot, const DealerFlag &flag);

 private:
  v::WorldDataView world;
  world::Field * field;

  double getScoreForFlag(v::RobotView robot, DealerFlag flag);
  std::vector<std::vector<double>> getScoreMatrix(const std::vector<v::RobotView> &allRobots, const FlagMap &flagMap);
  static double getFactorForPriority(const DealerFlag &flag);
  static double costForDistance(double distance, double fieldWidth, double fieldHeight);
  static double costForProperty(bool property);
  double scoreForFlags(const std::vector<Dealer::DealerFlag> &dealerFlags, const v::RobotView &robot);
  std::unordered_map<std::string, v::RobotView> mapFromAssignments(const std::vector<v::RobotView> &allRobots,
                                                            const FlagMap &flagMap,
                                                            const std::vector<int> &assignment) const;
};
}  // namespace rtt::ai
#endif  // RTT_ROBOTEAM_AI_SRC_UTILITIES_DEALER_H_
