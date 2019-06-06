//
// Created by mrlukasbos on 21-5-19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/control/RobotCommand.h>
#include <roboteam_ai/test/helpers/WorldHelper.h>

namespace rtt {

TEST(RobotCommandTest, it_converts_to_ros_command) {
    for (int i = 0; i < 100; i++) {
        RobotCommand rc;
        rc.id = (int) testhelpers::WorldHelper::getRandomValue(0, 15);
        rc.vel = testhelpers::WorldHelper::getRandomVelocity();
        rc.angle = testhelpers::WorldHelper::getRandomValue(0, 9.0);
        rc.kicker = (int) testhelpers::WorldHelper::getRandomValue(0, 2);
        rc.chipper = (int) testhelpers::WorldHelper::getRandomValue(0, 1);
        rc.geneva = (int) testhelpers::WorldHelper::getRandomValue(0, 5);
        rc.kickerVel = testhelpers::WorldHelper::getRandomValue(0, 6);
        rc.dribbler = (int) testhelpers::WorldHelper::getRandomValue(0, 32);
        auto cmd = rc.makeROSCommand();

        EXPECT_EQ(cmd.id, rc.id);
        EXPECT_FLOAT_EQ(cmd.x_vel, rc.vel.x);
        EXPECT_FLOAT_EQ(cmd.y_vel, rc.vel.y);
        EXPECT_FLOAT_EQ(cmd.w, rc.angle);
        EXPECT_EQ(cmd.kicker, rc.kicker);
        EXPECT_EQ(cmd.kicker_forced, rc.kickerForced);
        EXPECT_EQ(cmd.chipper, rc.chipper);
        EXPECT_EQ(cmd.chipper_forced, rc.chipperForced);
        EXPECT_EQ(cmd.geneva_state, rc.geneva);
        EXPECT_FLOAT_EQ(cmd.kicker_vel, rc.kickerVel);
        EXPECT_EQ(cmd.dribbler, rc.dribbler);
    }
}


} // rtt
