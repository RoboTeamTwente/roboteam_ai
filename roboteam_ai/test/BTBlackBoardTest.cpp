#include <gtest/gtest.h>
#include "../src/bt/bt.hpp"

roboteam_msgs::BoolEntry getBoolEntry(std::string name, bool value) {
  roboteam_msgs::BoolEntry be;
  be.name = name;
  be.value = value;
  return be;
}

roboteam_msgs::Int32Entry getIntEntry(std::string name, int value) {
  roboteam_msgs::Int32Entry ie;
  ie.name = name;
  ie.value = value;
  return ie;
}

roboteam_msgs::StringEntry getStringEntry(std::string name, std::string value) {
  roboteam_msgs::StringEntry se;
  se.name = name;
  se.value = value;
  return se;
}

TEST(BTBlackboardTest, blackboard) {

  roboteam_msgs::Blackboard bbmsg;

  bbmsg.bools.push_back(getBoolEntry("b_a", true));
  bbmsg.bools.push_back(getBoolEntry("b_b", false));

  bbmsg.ints.push_back(getIntEntry("i_a", 1000));
  bbmsg.ints.push_back(getIntEntry("i_b", -1000));
  bbmsg.ints.push_back(getIntEntry("i_c", 0));

  bbmsg.strings.push_back(getStringEntry("s_a", "roboteamtwente"));
  bbmsg.strings.push_back(getStringEntry("s_b", "is"));
  bbmsg.strings.push_back(getStringEntry("s_c", "awesome"));
  bbmsg.strings.push_back(getStringEntry("s_d", "!!!!11!!!!"));

  bt::Blackboard bb(bbmsg);

  ASSERT_EQ(bb.getBools().size(), 2);
  ASSERT_EQ(bb.getDoubles().size(), 0);
  ASSERT_EQ(bb.getInts().size(), 3);
  ASSERT_EQ(bb.getStrings().size(), 4);

  ASSERT_EQ(bb.GetBool("b_a"), true);
  ASSERT_EQ(bb.GetBool("b_b"), false);

  ASSERT_EQ(bb.GetString("s_a"), "roboteamtwente");
  ASSERT_EQ(bb.GetString("s_b"), "is");
  ASSERT_EQ(bb.GetString("s_c"), "awesome");
  ASSERT_EQ(bb.GetString("s_d"), "!!!!11!!!!");

  ASSERT_EQ(bb.GetInt("i_a"), 1000);
  ASSERT_EQ(bb.GetInt("i_b"), -1000);
  ASSERT_EQ(bb.GetInt("i_c"), 0);

  ASSERT_EQ(bb.HasBool("b_a"), true);
  ASSERT_EQ(bb.HasBool("b_d"), false);
  ASSERT_EQ(bb.HasBool("i_a"), false);
  ASSERT_EQ(bb.HasInt("i_a"), true);
  ASSERT_EQ(bb.HasInt("b_a"), false);
  ASSERT_EQ(bb.HasString("roboteamTwente"), false);
  ASSERT_EQ(bb.HasString("s_d"), true);

  bt::Blackboard bb2(bb.toMsg());
  ASSERT_EQ(bb.getStrings(), bb2.getStrings());
  ASSERT_EQ(bb.getInts(), bb2.getInts());
  ASSERT_EQ(bb.getDoubles(), bb2.getDoubles());
  ASSERT_EQ(bb.getBools(), bb2.getBools());

  bb.toTestX();
  bb.toString();
}


