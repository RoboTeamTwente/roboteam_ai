//
// Created by rolf on 26-09-20.
//

#include <control/positionControl/BBTrajectories/BBTrajectory1D.h>
#include <gtest/gtest.h>

namespace rtt::BB {
// A profile that only decellerates from the start
TEST(BBTrajectories, onlyDecelerate) {
    BBTrajectory1D test;
    double startPos = 0.0;
    double startVel = 1.0;
    double endPos = 1.0;
    double maxAcc = 0.5;
    double maxVel = 1.0;
    test.generateTrajectory(startPos, startVel, endPos, maxVel, maxAcc);
    EXPECT_DOUBLE_EQ(test.getTotalTime(), 2.0);
    EXPECT_DOUBLE_EQ(test.getPosition(0), startPos);
    EXPECT_DOUBLE_EQ(test.getPosition(-1), startPos);
    EXPECT_DOUBLE_EQ(test.getVelocity(1.0), 0.5);
    EXPECT_DOUBLE_EQ(test.getAcceleration(0.001), -maxAcc);
    EXPECT_DOUBLE_EQ(test.getVelocity(0), startVel);
    EXPECT_DOUBLE_EQ(test.getPosition(3), endPos);
    EXPECT_DOUBLE_EQ(test.getPosition(2), endPos);
    EXPECT_DOUBLE_EQ(test.getPosition(1), 0.75);
    EXPECT_DOUBLE_EQ(test.getVelocity(2), 0);
    EXPECT_DOUBLE_EQ(test.getVelocity(3), 0);
    EXPECT_TRUE(test.inLastPart(0.000));
    EXPECT_TRUE(test.inLastPart(1.5));
}
TEST(BBTrajectories, onlyDecelerateOtherWay) {
    BBTrajectory1D test;
    double startPos = 0.0;
    double startVel = -1.0;
    double endPos = -1.0;
    double maxAcc = 0.5;
    double maxVel = 1.0;
    test.generateTrajectory(startPos, startVel, endPos, maxVel, maxAcc);
    EXPECT_DOUBLE_EQ(test.getTotalTime(), 2.0);
    EXPECT_DOUBLE_EQ(test.getPosition(0), startPos);
    EXPECT_DOUBLE_EQ(test.getPosition(-1), startPos);
    EXPECT_DOUBLE_EQ(test.getVelocity(1.0), -0.5);
    EXPECT_DOUBLE_EQ(test.getAcceleration(0.001), maxAcc);
    EXPECT_DOUBLE_EQ(test.getVelocity(0), startVel);
    EXPECT_DOUBLE_EQ(test.getPosition(3), endPos);
    EXPECT_DOUBLE_EQ(test.getPosition(2), endPos);
    EXPECT_DOUBLE_EQ(test.getPosition(1), -0.75);
    EXPECT_DOUBLE_EQ(test.getVelocity(2), 0);
    EXPECT_DOUBLE_EQ(test.getVelocity(3), 0);
    // TODO: fix buggy parts
    EXPECT_TRUE(test.inLastPart(0.000));
    EXPECT_TRUE(test.inLastPart(1.5));
}
}  // namespace rtt::BB