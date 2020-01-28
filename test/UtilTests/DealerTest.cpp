#include <gtest/gtest.h>
#include "gmock/gmock.h"
#include <include/roboteam_ai/utilities/Dealer.h>

using namespace rtt::ai;
using namespace rtt::world_new::view;
using namespace rtt::world_new::robot;

using ::testing::Return;
using::testing::_;

class MockDealer : public rtt::ai::Dealer {
 public:
  MOCK_METHOD(double,
              getDefaultFlagScores,
              (const Data& data, const RobotView &robot, const Dealer::DealerFlag &flag),
              (override));
};

TEST(DealerTest, it_properly_distributes_robots) {
    // create a dealer whose 'getDefaultFlagScores' always returns 1;
    MockDealer dealer;
    ON_CALL(dealer, getDefaultFlagScores(_,_,_)).WillByDefault(Return(1));

    Dealer::DealerFlag highPriorityFlag(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::HIGH_PRIORITY);
    EXPECT_EQ(dealer.getFactorForPriority(highPriorityFlag), 3.0);
    Dealer::DealerFlag mediumPriorityFlag(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::MEDIUM_PRIORITY);
    EXPECT_EQ(dealer.getFactorForPriority(mediumPriorityFlag), 2.0);
    Dealer::DealerFlag lowPriorityFlag(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::LOW_PRIORITY);
    EXPECT_EQ(dealer.getFactorForPriority(lowPriorityFlag), 1.0);
}
