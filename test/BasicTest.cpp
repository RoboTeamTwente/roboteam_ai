#include <gtest/gtest.h>
#include "roboteam_world/robot.h"
#include "roboteam_world/ball.h"
#include <cmath>

namespace rtt {

TEST(BasicTests, robot) {
	Robot robot;
    ASSERT_EQ(INVALID_ROBOT_ID, robot.get_id());
    ASSERT_TRUE(std::isnan(robot.get_position().x));
    ASSERT_TRUE(std::isnan(robot.get_position().y));
    ASSERT_TRUE(std::isnan(robot.get_position().rot));

    robot.set_id(42);
    ASSERT_EQ(42, robot.get_id());

    robot.move_to(3.14, 100.1);
    ASSERT_FLOAT_EQ(3.14, robot.get_position().x);
    ASSERT_FLOAT_EQ(100.1, robot.get_position().y);

    robot.rotate_to(4.5);
    ASSERT_FLOAT_EQ(4.5, robot.get_position().rot);

    robot.set_vel(1.0, 2.0, static_cast<float>(-3.1415));
    ASSERT_FLOAT_EQ(1.0, robot.get_velocity().x);
    ASSERT_FLOAT_EQ(2.0, robot.get_velocity().y);
    ASSERT_FLOAT_EQ(-3.1415, robot.get_velocity().rot);
}

TEST(BasicTests, ball) {
    Ball ball;
    ASSERT_EQ(INVALID_AREA, ball.get_existence());
    ASSERT_TRUE(std::isnan(ball.get_position().x));
    ASSERT_TRUE(std::isnan(ball.get_position().y));
    ASSERT_TRUE(std::isnan(ball.get_position().rot));

    ball.set_area(4);
    ASSERT_EQ(4, ball.get_area());

    ball.move_to(1.0, -2.0, 3.1415);
    ASSERT_FLOAT_EQ(1.0, ball.get_position().x);
    ASSERT_FLOAT_EQ(-2.0, ball.get_position().y);
    ASSERT_FLOAT_EQ(3.1415, ball.get_position().rot);

    ball.set_velocity(1.2,3.4);
    ASSERT_FLOAT_EQ(1.2,ball.get_velocity().x);
    ASSERT_FLOAT_EQ(3.4,ball.get_velocity().y);

    ball.set_visible(false);
    auto y=ball.as_message();
    ASSERT_EQ(false,ball.as_message().visible);

}
}
