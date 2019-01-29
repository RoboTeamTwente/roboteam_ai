
#include <utility>

#include <gtest/gtest.h>

#include <roboteam_msgs/BoolEntry.h>
#include "roboteam_ai/src/bt/bt.hpp"

TEST(BTBlackBoardTest, Blackboard) {

    bt::Blackboard::Ptr bbmsg = std::make_shared<bt::Blackboard>();
    EXPECT_EQ(bbmsg->getBool("bool_a"), false);
    bbmsg->setBool("bool_a", true);
    bbmsg->setBool("bool_b", false);
    EXPECT_EQ(bbmsg->getBool("bool_a"), true);
    EXPECT_EQ(bbmsg->getBool("bool_b"), false);

    EXPECT_EQ(bbmsg->getInt("int_a"), - 1);
    bbmsg->setInt("int_a", 2);
    bbmsg->setInt("int_b", 0);
    bbmsg->setInt("int_c", 999999);
    EXPECT_EQ(bbmsg->getInt("int_a"), 2);
    EXPECT_EQ(bbmsg->getInt("int_b"), 0);
    EXPECT_EQ(bbmsg->getInt("int_c"), 999999);

    EXPECT_EQ(bbmsg->getDouble("double_a"), 0.0f);
    bbmsg->setDouble("double_a", 1);
    bbmsg->setDouble("double_b", - 3.141592653589793);
    bbmsg->setDouble("double_c", 1048576.201248);
    EXPECT_EQ(bbmsg->getDouble("double_a"), 1);
    EXPECT_EQ(bbmsg->getDouble("double_b"), - 3.141592653589793);
    EXPECT_EQ(bbmsg->getDouble("double_c"), 1048576.201248);

    EXPECT_EQ(bbmsg->getString("string_a"), "");
    bbmsg->setString("string_a", "RoboTeam Twente");
    bbmsg->setString("string_b", "is");
    bbmsg->setString("string_c", "awesome");
    bbmsg->setString("string_c", "awesome");
    bbmsg->setString("string_d", "!o!");
    EXPECT_EQ(bbmsg->getString("string_a"), "RoboTeam Twente");
    EXPECT_EQ(bbmsg->getString("string_b"), "is");
    EXPECT_EQ(bbmsg->getString("string_c"), "awesome");
    EXPECT_EQ(bbmsg->getString("string_d"), "!o!");

    EXPECT_EQ(bbmsg->getVector2("vector2_a"), rtt::Vector2(0, 0));
    bbmsg->setVector2("vector2_a", {1, 1});
    bbmsg->setVector2("vector2_b", {- 1.6180339887, 3.162277660168});
    bbmsg->setVector2("vector2_c", {42.0, - 42});
    EXPECT_EQ(bbmsg->getVector2("vector2_a").x, 1);
    EXPECT_EQ(bbmsg->getVector2("vector2_a").y, 1);
    EXPECT_EQ(bbmsg->getVector2("vector2_b").x, - 1.6180339887);
    EXPECT_EQ(bbmsg->getVector2("vector2_b").y, 3.162277660168);
    EXPECT_EQ(bbmsg->getVector2("vector2_c").x, 42.0);
    EXPECT_EQ(bbmsg->getVector2("vector2_c").y, - 42);

    EXPECT_EQ(bbmsg->hasBool("bool_a"), true);
    EXPECT_EQ(bbmsg->hasBool("bool_d"), false);
    EXPECT_EQ(bbmsg->hasBool("int_a"), false);
    EXPECT_EQ(bbmsg->hasInt("int_a"), true);
    EXPECT_EQ(bbmsg->hasInt("bool_a"), false);
    EXPECT_EQ(bbmsg->hasVector2("vector2_a"), true);
    EXPECT_EQ(bbmsg->hasDouble("double_a"), true);
    EXPECT_EQ(bbmsg->hasString("roboteam Twente"), false);
    EXPECT_EQ(bbmsg->hasString("awesome"), false);
    EXPECT_EQ(bbmsg->hasString("sting_b"), false);

    bbmsg->setVector2("vector2_c", {0, 0});
    bbmsg->setString("string_b", "is");

    EXPECT_EQ(bbmsg->getVector2("vector2_c").x, 0);
    EXPECT_EQ(bbmsg->getString("string_b"), "is");
}


