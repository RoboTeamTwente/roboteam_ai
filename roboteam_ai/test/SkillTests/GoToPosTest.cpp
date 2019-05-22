//
// Created by baris on 26/10/18.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/treeinterp/BTFactory.h>
#include "roboteam_ai/src/skills/gotopos/SkillGoToPos.h"
#include "roboteam_utils/Vector2.h"
#include "../../src/world/World.h"
#include "../../src/utilities/RobotDealer.h"

// Empty namespace for ROS errors
namespace {
std::vector<roboteam_msgs::RobotCommand> commands;

void robotCommandCallback(const roboteam_msgs::RobotCommandConstPtr &cmd) {
    commands.push_back(*cmd);
}

namespace w = rtt::ai::world;
namespace rd = rtt::ai::robotDealer;

TEST(SkillGoToPos, GoToPosTest) {
    //this test might fail about 50% of the time... no idea why
    rd::RobotDealer::halt();
    ros::Rate rate(1);
    commands.clear();
    EXPECT_TRUE(commands.empty());
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<roboteam_msgs::RobotCommand>(rtt::TOPIC_COMMANDS, 0, &robotCommandCallback);

    auto bb = std::make_shared<bt::Blackboard>();
    bb->setInt("ROBOT_ID", 0);
    bb->setString("ROLE", "GTPtest");
    bb->setVector2("targetPos", rtt::Vector2(5.0, 6.0));
    roboteam_msgs::World worldMsg;
    w::world->updateWorld(worldMsg);
    roboteam_msgs::WorldRobot robot;
    robot.id = 0;
    robot.pos.x = 0;
    robot.pos.y = 0;
    worldMsg.us.push_back(robot);

    roboteam_msgs::WorldRobot robot2;
    robot2.id = 1;
    robot2.pos.x = 10;
    robot2.pos.y = 10;
    worldMsg.us.push_back(robot2);

    worldMsg.ball.existence = 99999;
    w::world->updateWorld(worldMsg);
    rd::RobotDealer::claimRobotForTactic(rd::RobotType::RANDOM, "GTPtest", "GoToPosTest");
    rtt::ai::SkillGoToPos goToPos("GTPtest", bb);
    goToPos.initialize();
ros::spinOnce();
rate.sleep();
    if (goToPos.update() != bt::Leaf::Status::Running) {

}
    EXPECT_EQ(goToPos.update(), bt::Leaf::Status::Running);

    robot.pos.x = 5.0;
    robot.pos.y = 6.0;
    roboteam_msgs::World w2orldMsg;

    w2orldMsg.us.push_back(robot);
    w::world->updateWorld(w2orldMsg);

    EXPECT_EQ(goToPos.update(), bt::Leaf::Status::Success);

}
}
