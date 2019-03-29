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

    ApplicationManager app;
    app.setup();



    rate.sleep();
    ros::spinOnce();

    EXPECT_NE(app.strategy->getStatus(), bt::Node::Status::Waiting);

    app.notifyTreeStatus(bt::Node::Status::Waiting);
    app.notifyTreeStatus(bt::Node::Status::Running);
    app.notifyTreeStatus(bt::Node::Status::Success);
    EXPECT_EQ(ai::treeinterp::g_btfactory.getCurrentTree(), "haltStrategy");

    app.checkForShutdown();
    EXPECT_EQ(app.strategy->getStatus(), bt::Node::Status::Failure);
} // end of test


} // rtt
