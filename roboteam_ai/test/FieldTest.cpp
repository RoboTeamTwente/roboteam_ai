//
// Created by mrlukasbos on 19-10-18.
//

#include <gtest/gtest.h>
#include "../src/utilities/Field.h"
#include "helpers/WorldMessages.cpp"


TEST(FieldTest, it_gets_and_sets_the_field) {
    roboteam_msgs::GeometryFieldSize field;
    field.boundary_width = 42;
    rtt::ai::Field::set_field(field);
    ASSERT_EQ(rtt::ai::Field::get_field().boundary_width, 42);
}


TEST(FieldTest, it_gets_points_in_defence_area) {
    roboteam_msgs::GeometryFieldSize field;
	field.field_length = 8;
	field.field_width = 12;

	// set the penalty lines
	field.left_penalty_line.begin = rtt::Vector2(-4, 0);
	field.left_penalty_line.end = rtt::Vector2(-4, 8);
	field.right_penalty_line.begin = rtt::Vector2(4, 0);
	field.right_penalty_line.end = rtt::Vector2(4, 8);

  	// generate 100 random positions in our defence area
    for (int i=0; i<100; i++) { 
    	auto x = testhelpers::getRandomValue(-6, -4);
    	auto y = testhelpers::getRandomValue(0, 8);

    	// all points should be in our defence area
    	rtt::ai::Field::set_field(field);
    	bool inOurDefenceArea = rtt::ai::Field::pointIsInDefenceArea(rtt::Vector2(x, y), true, 0.0);
    	EXPECT_TRUE(inOurDefenceArea);

    	// the points should not be in their defence area
    	bool inTheirDefenceArea = rtt::ai::Field::pointIsInDefenceArea(rtt::Vector2(x, y), false, 0.0);
    	EXPECT_FALSE(inTheirDefenceArea);
    }

    // generate 100 random positions outside our defence area
    for (int i=0; i<100; i++) { 
    	auto x = testhelpers::getRandomValue(-4, 4);
    	auto y = testhelpers::getRandomValue(0, 8);
    	rtt::ai::Field::set_field(field);
    	bool inDefenceArea = rtt::ai::Field::pointIsInDefenceArea(rtt::Vector2(x, y), true, 0.0);
    	EXPECT_FALSE(inDefenceArea);
    }
}


TEST(FieldTest, it_returns_proper_goal_centers) {
    roboteam_msgs::GeometryFieldSize field;
	field.field_length = 8;
	field.field_width = 12;
    rtt::ai::Field::set_field(field);
    
    auto ourGoalCenter = rtt::ai::Field::get_our_goal_center();
    EXPECT_EQ(ourGoalCenter.x, -4);
    EXPECT_EQ(ourGoalCenter.y, 0);

    auto theirGoalCenter = rtt::ai::Field::get_their_goal_center();
    EXPECT_EQ(ourGoalCenter.x, 4);
    EXPECT_EQ(ourGoalCenter.y, 0);
}
