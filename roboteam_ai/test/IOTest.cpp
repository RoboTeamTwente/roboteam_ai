//
// Created by mrlukasbos on 24-10-18.
//

#include <gtest/gtest.h>
#include <roboteam_utils/Vector2.h>
#include "../src/io/IOManager.h"

// anonymous namespace needed to prevent ROS callback function name clashes
namespace {
roboteam_msgs::RobotCommand robotCommandFromCallback;

void robotCommandCallback(const roboteam_msgs::RobotCommandConstPtr &cmd) {
    robotCommandFromCallback = *cmd;
}

TEST(IOTest, it_subscribes) {
    ros::Rate rate(1);
    ros::NodeHandle nh;

    // subscribing
    ros::Publisher roleFeedbackPub = nh.advertise<roboteam_msgs::RoleFeedback>(rtt::TOPIC_ROLE_FEEDBACK, 1);
    ros::Publisher worldPub = nh.advertise<roboteam_msgs::World>(rtt::TOPIC_WORLD_STATE, 1);
    ros::Publisher geomPub = nh.advertise<roboteam_msgs::GeometryData>(rtt::TOPIC_GEOMETRY, 1);
    ros::Publisher refPub = nh.advertise<roboteam_msgs::RefereeData>(rtt::TOPIC_REFEREE, 1);

    rate.sleep();
    ros::spinOnce();

    EXPECT_EQ(robotCommandFromCallback.x_vel, 10);
    EXPECT_EQ(robotCommandFromCallback.y_vel, 20);

    // make an iomanager which subscribes to all and publishes to all
    rtt::ai::io::IOManager ioManager(true, true);

    // publish a world message
    roboteam_msgs::World worldMsg;
    worldMsg.ball.pos = rtt::Vector2(10, 20);
    worldPub.publish(worldMsg);

    // publish a geometry message
    roboteam_msgs::GeometryData geomMsg;
    geomMsg.field.goal_depth = 30;
    geomPub.publish(geomMsg);

    // publish role feedback message
    roboteam_msgs::RoleFeedback roleFeedbackMsg;
    roleFeedbackMsg.status = 'X';
    roleFeedbackPub.publish(roleFeedbackMsg);

    // publish role feedback message
    roboteam_msgs::RefereeData refData;
    refData.command.command = 3;
    refPub.publish(refData);

    rate.sleep();
    ros::spinOnce();

    EXPECT_EQ(ioManager.getWorldState().ball.pos.x, 10);
    EXPECT_EQ(ioManager.getWorldState().ball.pos.y, 20);
    EXPECT_EQ(ioManager.getGeometryData().field.goal_depth, 30);
    EXPECT_EQ(ioManager.getRoleFeedback().status, 'X');
    EXPECT_EQ(ioManager.getRefereeData().command.command, 3);

    // publishing
    ros::Subscriber robotCommandSub = nh.subscribe<roboteam_msgs::RobotCommand>(rtt::TOPIC_COMMANDS, 0,
            &robotCommandCallback);

    roboteam_msgs::RobotCommand cmd;
    cmd.x_vel = 10;
    cmd.y_vel = 20;
    ioManager.publishRobotCommand(cmd);

    rate.sleep();
    ros::spinOnce();

    EXPECT_EQ(robotCommandFromCallback.x_vel, 10);
    EXPECT_EQ(robotCommandFromCallback.y_vel, 20);
} // end of test
} // anonymous namespace