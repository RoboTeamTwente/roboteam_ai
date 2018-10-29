//
// Created by mrlukasbos on 24-10-18.
//

#include <gtest/gtest.h>
#include "../src/io/StrategyIOManager.h"
#include "../src/io/RoleIOManager.h"

// anonymous namespace needed to prevent ROS callback function name clashes
namespace {
roboteam_msgs::RobotCommand robotCommandFromCallback;
roboteam_msgs::RoleDirective roleDirectiveFromCallback;

void robotCommandCallback(const roboteam_msgs::RobotCommandConstPtr &cmd) {
    robotCommandFromCallback = *cmd;
}

void roleDirectiveCallback(const roboteam_msgs::RoleDirectiveConstPtr &roleDirective) {
    roleDirectiveFromCallback = *roleDirective;
}

TEST(IOTest, strategyIOManager) {
    // subscribing
    ros::Rate rate(1);
    ros::NodeHandle nh;

    ros::Publisher roleFeedbackPub = nh.advertise<roboteam_msgs::RoleFeedback>(rtt::TOPIC_ROLE_FEEDBACK, 1);
    ros::Publisher worldPub = nh.advertise<roboteam_msgs::World>(rtt::TOPIC_WORLD_STATE, 1);
    ros::Publisher geomPub = nh.advertise<roboteam_msgs::GeometryData>(rtt::TOPIC_GEOMETRY, 1);

    rtt::ai::io::StrategyIOManager strategyIOManager;
    strategyIOManager.subscribeToWorldState();
    strategyIOManager.subscribeToGeometryData();
    strategyIOManager.subscribeToRoleFeedback();

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

    rate.sleep();
    ros::spinOnce();

    EXPECT_EQ(strategyIOManager.getWorldState().ball.pos.x, 10);
    EXPECT_EQ(strategyIOManager.getWorldState().ball.pos.y, 20);
    EXPECT_EQ(strategyIOManager.getGeometryData().field.goal_depth, 30);
    EXPECT_EQ(strategyIOManager.getRoleFeedback().status, 'X');

    // publishing
    ros::Subscriber roleDirectiveSub = nh.subscribe<roboteam_msgs::RoleDirective>(rtt::TOPIC_ROLE_DIRECTIVE, 0,
            &roleDirectiveCallback);

    // publish roledirective
    roboteam_msgs::RoleDirective roleDirective;
    roleDirective.robot_id = 5;
    strategyIOManager.publishRoleDirective(roleDirective);

    rate.sleep();
    ros::spinOnce();

    EXPECT_EQ(roleDirectiveFromCallback.robot_id, 5);
}

TEST(IOTest, roleIOManager) {
    // subscribing
    ros::Rate rate(1);
    ros::NodeHandle nh;
    ros::Publisher roleDirectivePub = nh.advertise<roboteam_msgs::RoleDirective>(rtt::TOPIC_ROLE_DIRECTIVE, 1);
    ros::Publisher worldPub = nh.advertise<roboteam_msgs::World>(rtt::TOPIC_WORLD_STATE, 1);
    ros::Publisher geomPub = nh.advertise<roboteam_msgs::GeometryData>(rtt::TOPIC_GEOMETRY, 1);

    rtt::ai::io::RoleIOManager roleIOManager;
    roleIOManager.subscribeToWorldState();
    roleIOManager.subscribeToGeometryData();
    roleIOManager.subscribeToRoleDirective();

    // publish a world message
    roboteam_msgs::World worldMsg;
    worldMsg.ball.pos = rtt::Vector2(90, 100);
    worldPub.publish(worldMsg);

    // publish a geometry message
    roboteam_msgs::GeometryData geomMsg;
    geomMsg.field.goal_depth = 110;
    geomPub.publish(geomMsg);

    // publish a roledirective message
    roboteam_msgs::RoleDirective roleDirectiveMsg;
    roleDirectiveMsg.robot_id = 3;
    roleDirectivePub.publish(roleDirectiveMsg);

    rate.sleep();
    ros::spinOnce();

    EXPECT_EQ(roleIOManager.getWorldState().ball.pos.x, 90);
    EXPECT_EQ(roleIOManager.getWorldState().ball.pos.y, 100);
    EXPECT_EQ(roleIOManager.getGeometryData().field.goal_depth, 110);
    EXPECT_EQ(roleIOManager.getRoleDirective().robot_id, 3);

    // publishing
    ros::Subscriber robotCommandSub = nh.subscribe<roboteam_msgs::RobotCommand>(rtt::TOPIC_COMMANDS, 0,
            &robotCommandCallback);

    roboteam_msgs::RobotCommand cmd;
    cmd.x_vel = 10;
    cmd.y_vel = 20;
    roleIOManager.publishRobotCommand(cmd);

    rate.sleep();
    ros::spinOnce();

    EXPECT_EQ(robotCommandFromCallback.x_vel, 10);
    EXPECT_EQ(robotCommandFromCallback.y_vel, 20);
}
}