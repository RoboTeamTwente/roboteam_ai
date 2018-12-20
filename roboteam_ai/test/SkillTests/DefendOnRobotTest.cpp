//
// Created by robzelluf on 12/7/18.
//

#include "../../src/skills/DefendOnRobot.h"
#include "../../src/utilities/Constants.h"
#include <gtest/gtest.h>

TEST(DefendOnRobotTest, PositiveTest) {
    auto bb = std::make_shared<bt::Blackboard>();
    rtt::ai::DefendOnRobot DefendOnRobot("test", bb);
    DefendOnRobot.initialize();

    roboteam_msgs::WorldRobot robot1;
    robot1.pos.x = 0.0;
    robot1.pos.y = 0.0;
    robot1.angle = 0.0;

    roboteam_msgs::WorldRobot robot2;
    robot2.pos.x = 3.0;
    robot2.pos.y = 3.0;
    robot2.angle = static_cast<float>(-0.25 * M_PI);

    Vector2 newPosition = DefendOnRobot.calculateLocation();
    ASSERT_TRUE(newPosition.x > 0 && newPosition.x < 4);
    ASSERT_TRUE(newPosition.y > 0 && newPosition.y < 3);

    robot2.angle = static_cast<float>(0.25 * M_PI);
    newPosition = DefendOnRobot.calculateLocation();
    ASSERT_TRUE(newPosition.x > 3);
    ASSERT_TRUE(newPosition.y > 0);

    robot1.angle = static_cast<float>(-0.25 * M_PI);
    newPosition = DefendOnRobot.calculateLocation();
    std::cout << newPosition << std::endl;
    ASSERT_TRUE(newPosition.x > 3);
    ASSERT_TRUE(newPosition.y > 3);

    robot2.angle = static_cast<float>(-0.25 * M_PI);
    newPosition = DefendOnRobot.calculateLocation();
    std::cout << newPosition << std::endl;
    ASSERT_TRUE(newPosition.x < 3);
    ASSERT_TRUE(newPosition.y < 3);
}

TEST(DefendOnRobotTest, NegativeTest) {
    auto bb = std::make_shared<bt::Blackboard>();
    rtt::ai::DefendOnRobot DefendOnRobot("test", bb);
    DefendOnRobot.initialize();

    roboteam_msgs::WorldRobot robot1;
    robot1.pos.x = 0.0;
    robot1.pos.y = 0.0;
    robot1.angle = 0.0;

    roboteam_msgs::WorldRobot robot2;
    robot2.pos.x = 3.0;
    robot2.pos.y = static_cast<float>(-3.0);
    robot2.angle = static_cast<float>(0.25 * M_PI);

    Vector2 newPosition = DefendOnRobot.calculateLocation();
    ASSERT_TRUE(newPosition.x > 0 && newPosition.x < 4);
    ASSERT_TRUE(newPosition.y < 0 && newPosition.y > -3);

    robot2.angle = static_cast<float>(-0.25 * M_PI);
    newPosition = DefendOnRobot.calculateLocation();
    ASSERT_TRUE(newPosition.x > 3);
    ASSERT_TRUE(newPosition.y < 0);

    robot1.angle = static_cast<float>(-0.25 * M_PI);
    newPosition = DefendOnRobot.calculateLocation();
    std::cout << newPosition << std::endl;
    ASSERT_TRUE(newPosition.x > 3);
    ASSERT_TRUE(newPosition.y == 0);
}

