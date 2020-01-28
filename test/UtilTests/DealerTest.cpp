#include <gtest/gtest.h>
#include "gmock/gmock.h"
#include <include/roboteam_ai/utilities/Dealer.h>
#include <include/roboteam_ai/world_new/World.hpp>

using namespace rtt::ai;
using namespace rtt::world_new;
using namespace rtt::world_new::view;
using namespace rtt::world_new::robot;
using ::testing::Return;
using::testing::_;

class MockDealer : public rtt::ai::Dealer {
 public:
  explicit MockDealer(v::WorldDataView world) : Dealer(world, nullptr) {}

  MOCK_METHOD(double,
              getDefaultFlagScores,
              (const RobotView &robot, const Dealer::DealerFlag &flag),
              (override));
};

/*
 * Check if the robots are properly distributed
 */
TEST(DealerTest, it_properly_distributes_robots) {
    std::vector<proto::WorldRobot> yellowRobots;
    for (int i = 0; i < 11; i++) {
        proto::WorldRobot protoRobot;
        protoRobot.set_id(i);
        yellowRobots.push_back(protoRobot);
    }
    google::protobuf::RepeatedPtrField<proto::WorldRobot> data(yellowRobots.begin(), yellowRobots.end());
    proto::World protoWorld;
    protoWorld.mutable_yellow()->Swap(&data);
    World::instance()->updateWorld(protoWorld);

    // expect we are yellow and we have 11 robots
    EXPECT_EQ(World::instance()->getWorld()->getThem().size(), 11);

    // create a dealer whose 'getDefaultFlagScores' method always returns 1;
    MockDealer dealer(World::instance()->getWorld().value());
    ON_CALL(dealer, getDefaultFlagScores(_,_)).WillByDefault(Return(1));
}

/*
 * Check if the values of the priorities make sense.
 * High priority should give the highest value, Medium the medium etc.
 */
TEST(DealerTest, the_score_factor_increases_with_priority) {
    MockDealer dealer(World::instance()->getWorld().value());
    Dealer::DealerFlag highPriorityFlag(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag mediumPriorityFlag(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag lowPriorityFlag(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::LOW_PRIORITY);

    auto priority1 = dealer.getFactorForPriority(lowPriorityFlag);
    auto priority2 = dealer.getFactorForPriority(mediumPriorityFlag);
    auto priority3 = dealer.getFactorForPriority(highPriorityFlag);
    EXPECT_LT(priority1, priority2);
    EXPECT_LT(priority2, priority3);
}
