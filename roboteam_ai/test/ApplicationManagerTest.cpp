//
// Created by mrlukasbos on 14-1-19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/bt/bt.hpp>
#include <roboteam_ai/src/utilities/Referee.hpp>
#include "../src/ApplicationManager.h"

namespace rtt {
TEST(ApplicationManagerTest, it_handles_ROS_data) {
    ros::Rate rate(1);

    ApplicationManager app;
    app.setup();

    app.runOneLoopCycle();
    rate.sleep();
    EXPECT_NE(app.strategy->getStatus(), bt::Node::Status::Waiting);

    app.notifyTreeStatus(bt::Node::Status::Waiting);
    app.notifyTreeStatus(bt::Node::Status::Running);
    app.notifyTreeStatus(bt::Node::Status::Success);
    EXPECT_EQ(BTFactory::getCurrentTree(), "haltStrategy");

    app.checkForShutdown();
    EXPECT_EQ(app.strategy->getStatus(), bt::Node::Status::Failure);

} // end of test


} // rtt
