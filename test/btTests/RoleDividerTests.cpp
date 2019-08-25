//
// Created by mrlukasbos on 20-5-19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/bt/decorators/Succeeder.hpp>
#include "../../src/bt/RoleDivider.h"

TEST(roledividerTest, role_divider) {
    bt::RoleDivider rd;

    rd.giveProperty("test", "test1");
    EXPECT_EQ(rd.properties->getString("test"), "test1");
    EXPECT_TRUE(rd.getChildren().empty());
    std::shared_ptr<bt::Succeeder> child = std::make_shared<bt::Succeeder>();
    rd.addChild(child);
    EXPECT_FALSE(rd.getChildren().empty());
    EXPECT_EQ(rd.update(), bt::Node::Status::Waiting);
}