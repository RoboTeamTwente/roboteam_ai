#include <utility>

#include <utility>

#include <gtest/gtest.h>
#include <gtest/gtest_prod.h>

#include <roboteam_msgs/BoolEntry.h>
#include <roboteam_msgs/Int32Entry.h>
#include <roboteam_msgs/StringEntry.h>

#include "../src/bt/bt.hpp"


TEST(BTBlackBoardTest, Blackboard) {

    bt::Blackboard bbmsg;

    bbmsg.setBool("bool_a", true);
    bbmsg.setBool("bool_b", false);

    bbmsg.setInt("int_a", -1);
    bbmsg.setInt("int_b", 0);
    bbmsg.setInt("int_c", 999999);

    bbmsg.setDouble("double_a", 1);
    bbmsg.setDouble("double_b", -3.141592653589793);
    bbmsg.setDouble("double_c", 1048576.201248);

    bbmsg.setString("string_a", "RoboTeam Twente");
    bbmsg.setString("string_b", "is");
    bbmsg.setString("string_c", "awesome");
    bbmsg.setString("string_d", "!o!");

    bbmsg.setVector("vector2_a", {1,1});
    bbmsg.setVector("vector2_b", {-1.6180339887,3.162277660168});
    bbmsg.setVector("vector2_c", {42.0,-42});

    bt::Blackboard bb = bbmsg;

    ASSERT_EQ(bb.getBool("bool_a"), true);
    ASSERT_EQ(bb.getBool("bool_b"), false);

    ASSERT_EQ(bb.getInt("int_a"), -1);
    ASSERT_EQ(bb.getInt("int_b"), 0);
    ASSERT_EQ(bb.getInt("int_c"), 999999);

    ASSERT_EQ(bb.getDouble("double_a"), 1);
    ASSERT_EQ(bb.getDouble("double_b"), -3.141592653589793);
    ASSERT_EQ(bb.getDouble("double_c"), 1048576.201248);

    ASSERT_EQ(bb.getString("string_a"), "RoboTeam Twente");
    ASSERT_EQ(bb.getString("string_b"), "is");
    ASSERT_EQ(bb.getString("string_c"), "awesome");
    ASSERT_EQ(bb.getString("string_d"), "!o!");

    ASSERT_EQ(bb.getVector("vector2_a").x, 1);
    ASSERT_EQ(bb.getVector("vector2_a").y, 1);
    ASSERT_EQ(bb.getVector("vector2_b").x, -1.6180339887);
    ASSERT_EQ(bb.getVector("vector2_b").y, 3.162277660168);
    ASSERT_EQ(bb.getVector("vector2_c").x, 42.0);
    ASSERT_EQ(bb.getVector("vector2_c").y, -42);

    ASSERT_EQ(bb.hasBool("bool_a"), true);
    ASSERT_EQ(bb.hasBool("bool_d"), false);
    ASSERT_EQ(bb.hasBool("int_a"), false);
    ASSERT_EQ(bb.hasInt("int_a"), true);
    ASSERT_EQ(bb.hasInt("bool_a"), false);

    ASSERT_EQ(bb.hasString("roboteam Twente"), false);
    ASSERT_EQ(bb.hasString("awesome"), false);
    ASSERT_EQ(bb.hasString("sting_b"), false);

    bbmsg.setVector("vector2_c", {0,0});
    bbmsg.setString("string_b", "is");

    bt::Blackboard bb2 = bbmsg;

    ASSERT_EQ(bb2.getVector("vector2_c").x, 0);
    ASSERT_EQ(bb2.getString("string_b"), "is");

    ASSERT_EQ(bb.getString("string_a"), bb2.getString("string_a"));
    ASSERT_EQ(bb.getDouble("double_b"), bb2.getDouble("double_b"));

}


