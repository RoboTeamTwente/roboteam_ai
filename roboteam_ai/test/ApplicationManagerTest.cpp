//
// Created by mrlukasbos on 14-1-19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/bt/bt.hpp>
#include <roboteam_ai/src/utilities/Referee.hpp>
#include <roboteam_ai/src/utilities/Field.h>
#include "../src/ApplicationManager.h"
#include "../src/treeinterp/BTFactory.h"

namespace rtt {
TEST(ApplicationManagerTest, it_handles_ROS_data) {
    ros::Rate rate(1);
    ros::NodeHandle nh;

    // subscribing
    ros::Publisher worldPub = nh.advertise<roboteam_msgs::World>(rtt::TOPIC_WORLD_STATE, 1);
    ros::Publisher geomPub = nh.advertise<roboteam_msgs::GeometryData>(rtt::TOPIC_GEOMETRY, 1);
    ros::Publisher refPub = nh.advertise<roboteam_msgs::RefereeData>(rtt::TOPIC_REFEREE, 1);

    rate.sleep();
    ros::spinOnce();

    ApplicationManager app;
    app.setup();

    // publish a world message
    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot, robot1;
    robot.id = 0;
    robot.pos = rtt::Vector2(22, 13);
    robot.angle = 0.3;

    robot1.id = 0;
    worldMsg.us.push_back(robot1);
    worldMsg.ball.pos = rtt::Vector2(10.1, 20.2);
    worldMsg.ball.visible = 1;
    worldMsg.them.push_back(robot);

    worldPub.publish(worldMsg);

    // publish a geometry message
    roboteam_msgs::GeometryData geomMsg;
    geomMsg.field.field_length = 12;
    geomMsg.field.field_width = 9;
    geomMsg.field.goal_depth = 30.3;
    geomPub.publish(geomMsg);

    rate.sleep();
    ros::spinOnce();

    EXPECT_FLOAT_EQ(app.IOManager->getWorldState().ball.pos.x, 10.1);
    EXPECT_FLOAT_EQ(app.IOManager->getWorldState().ball.pos.y, 20.2);
    EXPECT_FLOAT_EQ(app.IOManager->getGeometryData().field.goal_depth, 30.3);

    EXPECT_EQ(app.worldMsg.ball.pos.x, 0);
    app.updateROSData();

    EXPECT_FLOAT_EQ(app.worldMsg.ball.pos.x, 10.1);
    EXPECT_FLOAT_EQ(app.worldMsg.ball.pos.y, 20.2);
    EXPECT_FLOAT_EQ(app.geometryMsg.field.goal_depth, 30.3);

    // run a loop
    // now the strategy should not be waiting anymore. (it should be running, failure or success)

    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 12;
    field.field_width = 9;

    ai::Field::set_field(field);
    app.runOneLoopCycle();
    rate.sleep();
    EXPECT_NE(app.strategy->getStatus(), bt::Node::Status::Waiting);

    app.notifyTreeStatus(bt::Node::Status::Waiting);
    app.notifyTreeStatus(bt::Node::Status::Running);
    app.notifyTreeStatus(bt::Node::Status::Success);
    EXPECT_EQ(ai::treeinterp::g_btfactory.getCurrentTree(), "haltStrategy");

    app.checkForShutdown();
    EXPECT_EQ(app.strategy->getStatus(), bt::Node::Status::Failure);

    df::DangerFinder::instance().stop();
} // end of test


} // rtt
