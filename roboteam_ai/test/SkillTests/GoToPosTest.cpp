//
// Created by baris on 26/10/18.
//

#include <gtest/gtest.h>
#include "../../src/skills/GoToPos.h"
#include "roboteam_utils/Vector2.h"
#include "../../src/utilities/World.h"
#include "../../src/utilities/RobotDealer.h"

// Empty namespace for ROS errors
namespace {
std::vector<roboteam_msgs::RobotCommand> commands;

void robotCommandCallback(const roboteam_msgs::RobotCommandConstPtr &cmd) {
    commands.push_back(*cmd);
}

TEST(GoToPos, GoToPosTest) {

    ros::Rate rate(1);
    commands.clear();
    EXPECT_TRUE(commands.empty());
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<roboteam_msgs::RobotCommand>(rtt::TOPIC_COMMANDS, 0, &robotCommandCallback);

    auto bb = std::make_shared<bt::Blackboard>();
    bb->setInt("ROBOT_ID", 1);
    bb->setString("ROLE","GTPtest");
    bb->setVector2("Position", rtt::Vector2(5.0,6.0));
    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;
    robot.id=1;
    robot.pos.x=0;
    robot.pos.y=0;
    worldMsg.us.push_back(robot);
    rtt::ai::World::set_world(worldMsg);
    robotDealer::RobotDealer::claimRobotForTactic(robotDealer::RobotType::random,"GoToPosTest","GTPtest");
    rtt::ai::GoToPos goToPos("GTPtest", bb);
    goToPos.initialize();

    EXPECT_EQ(goToPos.update(), bt::Leaf::Status::Running);

    // Wait a little bit
    rate.sleep();
    ros::spinOnce();

    std::vector<roboteam_msgs::RobotCommand> cmds = commands;
    EXPECT_EQ((signed int) commands.size(), 1);
    EXPECT_TRUE(commands.at(0).x_vel);
    EXPECT_TRUE(commands.at(0).y_vel);
    robotDealer::RobotDealer::removeTactic("GoToPosTest");

}
}