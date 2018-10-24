// subscribe to the robotcommands channel
// run the kick skill
// check if the commands are on the robotcommands channel.

#include "ros/ros.h"
#include "../../src/io/RoleIOManager.h"
#include "../../src/skills/Kick.h"
#include <gtest/gtest.h>
#include <roboteam_utils/constants.h>
#include <roboteam_msgs/RobotCommand.h>

std::vector<roboteam_msgs::RobotCommand> commands;

void robotCommandCallback(const roboteam_msgs::RobotCommandConstPtr &cmd) {
  commands.push_back(*cmd);
}

TEST(KickTest, It_sends_proper_robotcommands) {
  ros::Rate rate(1);
  commands.clear(); // ensure the vector is empty.
  ASSERT_TRUE(commands.empty());
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<roboteam_msgs::RobotCommand>(rtt::TOPIC_COMMANDS, 0, &robotCommandCallback);

  auto bb = std::make_shared<bt::Blackboard>();
  bb->SetInt("ROBOT_ID", 1);
  rtt::ai::Kick kick("test", bb);
  kick.Initialize();

  ASSERT_EQ(kick.Update(), bt::Leaf::Status::Running);

  // wait a little for the message to arrive and then spin
  rate.sleep();
  ros::spinOnce();

  std::vector<roboteam_msgs::RobotCommand> cmds = commands;
  ASSERT_EQ(commands.size(), 1);
  ASSERT_EQ(commands.at(0).kicker_vel, DEFAULT_KICK_POWER);
}

