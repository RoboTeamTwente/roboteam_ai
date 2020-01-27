#include <gtest/gtest.h>
#include <include/roboteam_ai/utilities/Dealer.h>

TEST(DealerTest, it_claims_and_frees) {
   std::vector robotIds = {1,2,3,4,5,6,7,8,9,10,11};
   rtt::ai::Dealer dealer;
   dealer.claimRobot(robotIds, "role_1", {}, false);
   EXPECT_EQ(dealer.getClaimedRoles().size(), 1);

    dealer.claimRobot(robotIds, "role_1", {}, false);
    EXPECT_EQ(dealer.getClaimedRoles().size(), 1);

    dealer.claimRobot(robotIds, "role_2", {}, false);
    EXPECT_EQ(dealer.getClaimedRoles().size(), 2);

    dealer.freeRobot("role_3");
    EXPECT_EQ(dealer.getClaimedRoles().size(), 2);

    dealer.freeRobot("role_1");
    EXPECT_EQ(dealer.getClaimedRoles().size(), 1);

    dealer.freeAllRobots();
    EXPECT_EQ(dealer.getClaimedRoles().size(), 0);
}
