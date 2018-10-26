//
// Created by baris on 26/10/18.
//

#include <gtest/gtest.h>
#include "../../src/skills/GoToPos.h"
#include "ros/ros.h"

// Empty namespace for ROS errors
namespace {
std::vector<roboteam_msgs::RobotCommand> commands;

void robotCommandCallback(const roboteam_msgs::RobotCommandConstPtr &cmd) {
    commands.push_back(*cmd);
}

TEST(GoTOPos, GoTOPosTest) {

    ros::Rate rate(1);
    commands.clear();
    EXPECT_TRUE(commands.empty());
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<roboteam_msgs::RobotCommand>(rtt::TOPIC_COMMANDS, 0, &robotCommandCallback);

    auto bb = std::make_shared<bt::Blackboard>();
    bb->SetInt("ROBOT_ID", 1);
    bb->SetInt("X", 5);
    bb->SetInt("Y", 6);
    rtt::ai::GoToPos goToPos("test1", bb);
    goToPos.Initialize();

    EXPECT_EQ(goToPos.Update(), bt::Leaf::Status::Running);

    // Wait a little bit
    rate.sleep();
    ros::spinOnce();

    std::vector<roboteam_msgs::RobotCommand> cmds = commands;
    EXPECT_EQ(commands.size(), 1);
    EXPECT_TRUE(commands.at(0).x_vel);
    EXPECT_TRUE(commands.at(0).y_vel);



}
}