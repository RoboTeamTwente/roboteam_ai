//
// Created by thijs on 25-10-18.
//


// subscribe to the robotcommands channel
// run the kick skill
// check if the commands are on the robotcommands channel.

#include "ros/ros.h"
#include "../../src/io/RoleIOManager.h"
#include "../../src/skills/Rotate.h"
#include <gtest/gtest.h>
#include <roboteam_utils/constants.h>
#include <roboteam_msgs/RobotCommand.h>

// anonymous namespace needed to prevent ROS callback function name clashes
namespace {
roboteam_msgs::GeometryFieldSize fieldMsg;

void setFieldtoWorld() {
    // set the field parameters
    fieldMsg.field_length = 1000;
    fieldMsg.field_width = 500;
    fieldMsg.goal_width = 200;
    fieldMsg.goal_depth = 20;

    // set the field to the world
    rtt::ai::Field::set_field(fieldMsg);
}

roboteam_msgs::WorldRobot getRobot(int x, int y, float angle, int id = 0) {
    roboteam_msgs::WorldRobot robot;
    robot.pos = rtt::Vector2(x, y);
    robot.id = (unsigned int) id;
    robot.angle = angle;
    return robot;
}

// return a ball at a given location
roboteam_msgs::WorldBall getBall(int x, int y) {
    roboteam_msgs::WorldBall ball;
    ball.pos = rtt::Vector2(x, y);
    return ball;
}

std::vector<roboteam_msgs::RobotCommand> commands;

void robotCommandCallback(const roboteam_msgs::RobotCommandConstPtr &cmd) {
    commands.push_back(*cmd);
}
//TODO: FIX TEST \o/
TEST(RotateTest, it_rotates) {

    roboteam_msgs::World worldMsg;
    setFieldtoWorld();

    worldMsg.ball = getBall(-100, 0);
    worldMsg.us.push_back(getRobot(10, 200, (float)PI/8,1));


    ros::Rate rate(1);
    commands.clear(); // ensure the vector is empty.
    EXPECT_TRUE(commands.empty());
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<roboteam_msgs::RobotCommand>(rtt::TOPIC_COMMANDS, 0, &robotCommandCallback);

    auto bb = std::make_shared<bt::Blackboard>();

    bb->SetBool("Rotate_To_Object",true);
    bb->SetInt("Rotate_Object",100);        // Rotate to ball
    bb->SetFloat("Rotate_Angle", (float)PI/2);

    rtt::ai::Rotate rotateOne;
    rotateOne.Initialize();
    bt::Node::Status statusOne = rotateOne.Update();
    EXPECT_EQ(statusOne, bt::Node::Status::Running);

    rate.sleep();
    ros::spinOnce();

    EXPECT_EQ(commands.at(0).w, -MAX_ANGULAR_VELOCITY);

    commands.clear(); // ensure the vector is empty.

    bb->SetBool("Rotate_To_Object",true);
    bb->SetInt("Rotate_Object",101);        // Rotate to center of the enemy goal

    rtt::ai::Rotate rotateTwo;
    rotateTwo.Initialize();
    rotateTwo.Update();

    rate.sleep();
    ros::spinOnce();

    EXPECT_EQ(commands.at(0).w, MAX_ANGULAR_VELOCITY);

    commands.clear();

    bb->SetBool("Rotate_To_Object",false);
    bb->SetFloat("Rotate_Angle", (float)PI/4);

    rtt::ai::Rotate rotateThree;
    rotateThree.Initialize();
    rotateThree.Update();

    rate.sleep();
    ros::spinOnce();

    EXPECT_EQ(commands.at(0).w, MAX_ANGULAR_VELOCITY);
}

}

