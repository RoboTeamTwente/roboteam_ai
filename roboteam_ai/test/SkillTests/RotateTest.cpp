//
// Created by thijs on 25-10-18.
//


// subscribe to the robotcommands channel
// run the kick skill
// check if the commands are on the robotcommands channel.

#include "ros/ros.h"
#include "../../src/skills/Rotate.h"

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

roboteam_msgs::WorldRobot setRobot(int x, int y, float angle, int id = 0) {
    roboteam_msgs::WorldRobot robot;
    robot.pos = rtt::Vector2(x, y);
    robot.id = (unsigned int) id;
    robot.angle = angle;
    return robot;
}

// return a ball at a given location
roboteam_msgs::WorldBall setBall(int x, int y) {
    roboteam_msgs::WorldBall ball;
    ball.pos = rtt::Vector2(x, y);
    return ball;
}

std::vector<roboteam_msgs::RobotCommand> commands;

void robotCommandCallback(const roboteam_msgs::RobotCommandConstPtr &cmd) {
    commands.push_back(*cmd);
}
//TODO: FIX TEST \o/ FIX BLACKBOARDS / ROBOT ID / segmentation faults :o
TEST(RotateTest, It_rotates) {

    roboteam_msgs::World worldMsg;
    setFieldtoWorld();

    worldMsg.ball = setBall(100, 100);
    worldMsg.us.push_back(setRobot(- 100, - 100, (float) (0.625*rtt::ai::constants::PI), 1));
    rtt::ai::World::set_world(worldMsg);

    ros::Rate rate(1);
    commands.clear(); // ensure the vector is empty.
    EXPECT_TRUE(commands.empty());
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<roboteam_msgs::RobotCommand>(rtt::TOPIC_COMMANDS, 0, &robotCommandCallback);

    auto bb = std::make_shared<bt::Blackboard>();

    bb->setInt("ROBOT_ID", 1);
    bb->setBool("Rotate_To_Object", true);
    bb->setInt("Rotate_Object", 100);        // Rotate to ball
    bb->setFloat("Rotate_Angle", (float) (rtt::ai::constants::PI*0.5));

    rtt::ai::Rotate rotateOne("test1", bb);
    rotateOne.Initialize();
    bt::Node::Status statusOne = rotateOne.update();
    EXPECT_EQ(statusOne, bt::Node::Status::Running);

    rate.sleep();
    ros::spinOnce();

    EXPECT_EQ(commands.at(0).w, rtt::ai::constants::MAX_ANGULAR_VELOCITY);

    //commands.clear(); // ensure the vector is empty.

    bb->setBool("Rotate_To_Object", true);
    bb->setInt("Rotate_Object", 102);        // Rotate to center of the enemy goal

    rtt::ai::Rotate rotateTwo("test2", bb);
    rotateTwo.initialize();
    bt::Node::Status statusTwo = rotateOne.update();
    EXPECT_EQ(statusTwo, bt::Node::Status::Running);

    rate.sleep();
    ros::spinOnce();

    EXPECT_EQ(commands.at(1).w, rtt::ai::constants::MAX_ANGULAR_VELOCITY);

    bb->setBool("Rotate_To_Object", false);
    bb->setFloat("Rotate_Angle", (float) - rtt::ai::constants::PI);

    rtt::ai::Rotate rotateThree("test3", bb);
    rotateThree.initialize();
    bt::Node::Status statusThree = rotateOne.update();
    EXPECT_EQ(statusThree, bt::Node::Status::Running);

    rate.sleep();
    ros::spinOnce();

    EXPECT_EQ(commands.at(2).w, rtt::ai::constants::MAX_ANGULAR_VELOCITY);
}
} // anonymous namespace

