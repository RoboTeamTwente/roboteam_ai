//
// Created by mrlukasbos on 14-1-19.
//

#include <gtest/gtest.h>
#include "../src/ApplicationManager.h"

namespace rtt {
TEST(ApplicationManagerTest, it_runs_the_main_loop)
{
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
    worldMsg.ball.pos = rtt::Vector2(10.1, 20.2);
    worldPub.publish(worldMsg);

    // publish a geometry message
    roboteam_msgs::GeometryData geomMsg;
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

    ASSERT_EQ(app.factory.getCurrentTree(), "SimpleStrategy");
} // end of test
} // rtt
