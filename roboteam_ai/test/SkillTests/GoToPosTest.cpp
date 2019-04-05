//
// Created by baris on 26/10/18.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/treeinterp/BTFactory.h>
#include "../../src/skills/GoToPos.h"
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

TEST(GoToPos, GoToPosTest) {

    ros::Rate rate(1);
    commands.clear();
    EXPECT_TRUE(commands.empty());
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<roboteam_msgs::RobotCommand>(rtt::TOPIC_COMMANDS, 0, &robotCommandCallback);

    auto bb = std::make_shared<bt::Blackboard>();
    bb->setInt("ROBOT_ID", 0);
    bb->setString("ROLE","GTPtest");
    bb->setVector2("targetPos", rtt::Vector2(5.0,6.0));
    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;
    robot.id=0;
    robot.pos.x=0;
    robot.pos.y=0;
    worldMsg.us.push_back(robot);
    worldMsg.ball.existence = 99999;
    w::world->updateWorld(worldMsg);
    rd::RobotDealer::claimRobotForTactic(rd::RobotType::RANDOM,"GoToPosTest","GTPtest");
    rtt::ai::GoToPos goToPos("GTPtest", bb);
    goToPos.initialize();

    EXPECT_EQ(goToPos.targetPos, bb->getVector2("targetPos"));
    EXPECT_EQ(goToPos.update(), bt::Leaf::Status::Running);

    robot.pos.x = 5.0;
    robot.pos.y = 6.0;

    worldMsg.us[0] = robot;
    w::world->updateWorld(worldMsg);

    EXPECT_EQ(goToPos.update(), bt::Leaf::Status::Success);

}
}
