#include <gtest/gtest.h>
#include "roboteam_world/robot.h"
#include "roboteam_world/ball.h"
#include <cmath>

namespace rtt {

TEST(BasicTests, robot) {
	Robot rob;
    ASSERT_EQ(INVALID_ROBOT_ID, rob.get_id());
    ASSERT_TRUE(std::isnan(rob.get_position().x));
    ASSERT_TRUE(std::isnan(rob.get_position().y));
    ASSERT_TRUE(std::isnan(rob.get_position().rot));

    rob.set_id(42);
    ASSERT_EQ(42, rob.get_id());

    rob.move_to(3.14, 100.1);
    ASSERT_FLOAT_EQ(3.14, rob.get_position().x);
    ASSERT_FLOAT_EQ(100.1, rob.get_position().y);

    rob.rotate_to(4.5);
    ASSERT_FLOAT_EQ(4.5, rob.get_position().rot);

    rob.set_vel(1.0, 2.0, -3.1415);
    ASSERT_FLOAT_EQ(1.0, rob.get_velocity().x);
    ASSERT_FLOAT_EQ(2.0, rob.get_velocity().y);
    ASSERT_FLOAT_EQ(-3.1415, rob.get_velocity().rot);
}

TEST(BasicTests, ball) {
    Ball ball;
    ASSERT_EQ(INVALID_AREA, ball.get_existence());
    ASSERT_TRUE(std::isnan(ball.get_position().x));
    ASSERT_TRUE(std::isnan(ball.get_position().y));
    ASSERT_TRUE(std::isnan(ball.get_position().rot));

    ball.set_existence(4);
    ASSERT_EQ(4, ball.get_existence());

    ball.move_to(1.0, -2.0, 3.1415);
    ASSERT_FLOAT_EQ(1.0, ball.get_position().x);
    ASSERT_FLOAT_EQ(-2.0, ball.get_position().y);
    ASSERT_FLOAT_EQ(3.1415, ball.get_position().rot);
}

}
