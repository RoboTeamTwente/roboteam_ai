#include <gtest/gtest.h>
#include <include/roboteam_ai/utilities/Dealer.h>
#include <include/roboteam_ai/world_new/World.hpp>
#include "../helpers/WorldHelper.h"
#include "../helpers/FieldHelper.h"

using namespace rtt::ai;
using namespace rtt::world_new;
using namespace rtt::world_new::view;
using namespace rtt::world_new::robot;

/*
 * @brief Fixture for the dealing test
 *
 * Every Test case will be booted with a world with 3+1 robots and no ball.
 */
class DealerTest : public ::testing::Test {
 protected:
  // DO NOT OVERRIDE STYLING ACCORDING TO CLANG TIDY
  virtual void SetUp(){
      int yellow = 1, blue = 3;
      if (World::instance()->getSettings().isYellow()) {
          yellow = 3;
          blue = 1;
      }
      auto protoWorld = testhelpers::WorldHelper::getWorldMsg(yellow,blue,false,testhelpers::FieldHelper::generateField());
      World::instance()->updateWorld(protoWorld);
  }
};

/*
 * @brief Mock class for Dealer.
 *
 * The getDefaultFlagScores() function can be overridden,
 * Since it brings in a lot of uncertainty from world and field.
 */
class MockDealer : public rtt::ai::Dealer {
 public:
  explicit MockDealer(v::WorldDataView world) : Dealer(world, nullptr) {}
  double getDefaultFlagScores(const RobotView &robot, const Dealer::DealerFlag &flag) override {
      return robot->getId() + 1;
  }
  // MOCK_METHOD(double, getDefaultFlagScores, (const RobotView &robot, const Dealer::DealerFlag &flag), (override));
};

/*
 * Check if the robots are properly distributed
 */
TEST_F(DealerTest, it_properly_distributes_robots) {
    // create a dealer whose 'getDefaultFlagScores' method always returns robot id;
    MockDealer dealer(World::instance()->getWorld().value());

    Dealer::FlagMap flagMap;
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);

    flagMap.insert({"test_role_0", {closeToBallFlag}});
    flagMap.insert({"test_role_1", {closeToTheirGoalFlag}});
    flagMap.insert({"test_role_2", {closeToTheirGoalFlag, closeToBallFlag}});

    auto matrix = dealer.getScoreMatrix(World::instance()->getWorld()->getUs(), flagMap);
    EXPECT_EQ(matrix.size(), 3); // columns
    EXPECT_EQ(matrix.at(0).size(), 3); // rows

    // the values inside the matrix (format: matrix[role][robotId])
    EXPECT_DOUBLE_EQ(matrix[0][0], 3);
    EXPECT_DOUBLE_EQ(matrix[0][1], 6);
    EXPECT_DOUBLE_EQ(matrix[0][2], 9);
    EXPECT_DOUBLE_EQ(matrix[1][0], 2);
    EXPECT_DOUBLE_EQ(matrix[1][1], 4);
    EXPECT_DOUBLE_EQ(matrix[1][2], 6);
    EXPECT_DOUBLE_EQ(matrix[2][0], 5);
    EXPECT_DOUBLE_EQ(matrix[2][1], 10);
    EXPECT_DOUBLE_EQ(matrix[2][2], 15);

    // these values should correspons with the scores above.
    auto distribution = dealer.distribute(World::instance()->getWorld()->getUs(), flagMap);
    EXPECT_EQ(distribution.at("test_role_0")->getId(), 1);
    EXPECT_EQ(distribution.at("test_role_1")->getId(), 2);
    EXPECT_EQ(distribution.at("test_role_2")->getId(), 0);
}

/*
 * Check if the values of the priorities make sense.
 * High priority should give the highest value, Medium the medium etc.
 */
TEST_F(DealerTest, the_score_factor_increases_with_priority) {
    MockDealer dealer(World::instance()->getWorld().value());
    Dealer::DealerFlag highPriorityFlag(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag mediumPriorityFlag(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag lowPriorityFlag(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::LOW_PRIORITY);

    auto priority1 = MockDealer::getFactorForPriority(lowPriorityFlag);
    auto priority2 = MockDealer::getFactorForPriority(mediumPriorityFlag);
    auto priority3 = MockDealer::getFactorForPriority(highPriorityFlag);
    EXPECT_LT(priority1, priority2);
    EXPECT_LT(priority2, priority3);
}
