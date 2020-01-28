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

    // create a dealer whose 'getDefaultFlagScores' method always returns 1;
    MockDealer dealer;
    ON_CALL(dealer, getDefaultFlagScores(_,_,_)).WillByDefault(Return(1));


}

/*
 * Check if the values of the priorities make sense.
 * High priority should give the highest value, Medium the medium etc.
 */
TEST(DealerTest, the_score_factor_increases_with_priority) {
    MockDealer dealer;
    Dealer::DealerFlag highPriorityFlag(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag mediumPriorityFlag(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag lowPriorityFlag(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::LOW_PRIORITY);

    auto priority1 = dealer.getFactorForPriority(lowPriorityFlag);
    auto priority2 = dealer.getFactorForPriority(mediumPriorityFlag);
    auto priority3 = dealer.getFactorForPriority(highPriorityFlag);
    EXPECT_LT(priority1, priority2);
    EXPECT_LT(priority2, priority3);
}
