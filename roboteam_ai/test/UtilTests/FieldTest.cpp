//
// Created by mrlukasbos on 19-10-18.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/world/World.h>
#include "roboteam_ai/src/world/Field.h"
#include "roboteam_ai/test/helpers/WorldHelper.h"

TEST(FieldTest, it_gets_and_sets_the_field) {
    roboteam_msgs::GeometryFieldSize field;
    field.boundary_width = 42;
    rtt::ai::world::field->set_field(field);
    EXPECT_EQ(rtt::ai::world::field->get_field().boundary_width, 42);
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
    	auto x = testhelpers::WorldHelper::getRandomValue(-6, -4);
    	auto y = testhelpers::WorldHelper::getRandomValue(0, 8);

    	// all points should be in our defence area
    	rtt::ai::world::field->set_field(field);
    	bool inOurDefenceArea = rtt::ai::world::field->pointIsInDefenceArea(rtt::Vector2(x, y), true, 0.0);
    	EXPECT_TRUE(inOurDefenceArea);

    	// the points should not be in their defence area
    	bool inTheirDefenceArea = rtt::ai::world::field->pointIsInDefenceArea(rtt::Vector2(x, y), false, 0.0);
    	EXPECT_FALSE(inTheirDefenceArea);
    }

    // generate 100 random positions outside our defence area
    for (int i=0; i<100; i++) { 
    	auto x = testhelpers::WorldHelper::getRandomValue(-4, 4);
    	auto y = testhelpers::WorldHelper::getRandomValue(0, 8);
    	rtt::ai::world::field->set_field(field);
    	bool inDefenceArea = rtt::ai::world::field->pointIsInDefenceArea(rtt::Vector2(x, y), true, 0.0);
    	EXPECT_FALSE(inDefenceArea);
    }
}


TEST(FieldTest, it_returns_proper_goal_centers) {
    roboteam_msgs::GeometryFieldSize field;
	field.field_length = 8;
	field.field_width = 12;
    rtt::ai::world::field->set_field(field);
    
    auto ourGoalCenter = rtt::ai::world::field->get_our_goal_center();
    EXPECT_EQ(ourGoalCenter.x, -4);
    EXPECT_EQ(ourGoalCenter.y, 0);

    auto theirGoalCenter = rtt::ai::world::field->get_their_goal_center();
    EXPECT_EQ(theirGoalCenter.x, 4);
    EXPECT_EQ(theirGoalCenter.y, 0);
}


TEST(FieldTest, it_detects_points_in_field_properly) {
	roboteam_msgs::GeometryFieldSize field;
	field.field_length = 8;
	field.field_width = 12;
	rtt::ai::world::field->set_field(field);

	rtt::Vector2 point;
	
	// Middle
	point = {0, 0};
	EXPECT_TRUE(rtt::ai::world::field->pointIsInField(point));

	// exactly on the edge
	point = {4, 6};
	EXPECT_FALSE(rtt::ai::world::field->pointIsInField(point));
	EXPECT_TRUE(rtt::ai::world::field->pointIsInField(point, -0.1)); // expect true if it has a margin outside the field

	// exactly on the edge, negative
	point = {-4, -6};
	EXPECT_FALSE(rtt::ai::world::field->pointIsInField(point));
	EXPECT_TRUE(rtt::ai::world::field->pointIsInField(point, -0.1)); // expect true if it has a margin outside the field

	// exactly on the edge, negative
	point = {-3.8, -5.8};
	EXPECT_TRUE(rtt::ai::world::field->pointIsInField(point));
	EXPECT_FALSE(rtt::ai::world::field->pointIsInField(point, 0.3)); // expect false if it has a margin inside the field
}

TEST(FieldTest, it_calculates_obstacles) {
	roboteam_msgs::GeometryFieldSize field;
	field.field_length = 8;
	field.field_width = 12;
	field.goal_width = 1;
	rtt::ai::world::field->set_field(field);
	roboteam_msgs::WorldRobot robot;
	robot.id = 0;

	// watch our goal from the center of the field
	// there are no robots in between
	auto world = testhelpers::WorldHelper::getWorldMsg(0, 0, false, field);
	robot.pos = rtt::Vector2(0, 0);
	world.us.push_back(robot);
	rtt::ai::world::world->setWorld(world);
	auto obstacles = rtt::ai::world::field->getBlockadesMappedToGoal(true, {0, 0});
	auto visibleParts = rtt::ai::world::field->getVisiblePartsOfGoal(true, {0, 0});
	EXPECT_EQ(obstacles.size(), 0);
	EXPECT_EQ(visibleParts.size(), 1);
	EXPECT_FLOAT_EQ(visibleParts.at(0).first.dist(visibleParts.at(0).second), field.goal_width);
	EXPECT_EQ(rtt::ai::world::field->getPercentageOfGoalVisibleFromPoint(true, {0, 0}), 100);


	// watch our goal from the center of the field
	// there are no robots in between
	world = testhelpers::WorldHelper::getWorldMsg(0, 0, false, field);
	robot.pos = rtt::Vector2(-4, 0);
	world.us.push_back(robot);
	rtt::ai::world::world->setWorld(world);
	obstacles = rtt::ai::world::field->getBlockadesMappedToGoal(true, {0, 0});
	visibleParts = rtt::ai::world::field->getVisiblePartsOfGoal(true, {0, 0});
	EXPECT_EQ(obstacles.size(), 1);
	EXPECT_EQ(visibleParts.size(), 2);
	// the width should be somewhere equal to the width of the robot
	EXPECT_FLOAT_EQ(rtt::ai::world::field->getPercentageOfGoalVisibleFromPoint(true, {0, 0}), round(100 - 200*rtt::ai::Constants::ROBOT_RADIUS()));
	EXPECT_FLOAT_EQ(obstacles.at(0).first.dist(obstacles.at(0).second), 2*rtt::ai::Constants::ROBOT_RADIUS()); // the width of the obstacle is twice robot radius

	// watch their goal from the center of the field
	// there are two robots in between, separated
	roboteam_msgs::WorldRobot robot2;
	robot2.id = 1;

	world = testhelpers::WorldHelper::getWorldMsg(0, 0, false, field);
	robot.pos = rtt::Vector2(4, 0);
	robot2.pos = rtt::Vector2(4, 0.3);
	world.us.push_back(robot);
	world.us.push_back(robot2);
	rtt::ai::world::world->setWorld(world);
	
	obstacles = rtt::ai::world::field->getBlockadesMappedToGoal(false, {0, 0});
	visibleParts = rtt::ai::world::field->getVisiblePartsOfGoal(false, {0, 0});
	EXPECT_EQ(obstacles.size(), 2);
	EXPECT_EQ(visibleParts.size(), 3);
	EXPECT_FLOAT_EQ(rtt::ai::world::field->getPercentageOfGoalVisibleFromPoint(false, {0, 0}), round(100 - 400*rtt::ai::Constants::ROBOT_RADIUS()));

	// watch their goal from the center of the field
	// there are two robots in between, merged
	world = testhelpers::WorldHelper::getWorldMsg(0, 0, false, field);
	robot.pos = rtt::Vector2(4, 0);
	robot2.pos = rtt::Vector2(4, 0.05);
	world.us.push_back(robot);
	world.us.push_back(robot2);
	rtt::ai::world::world->setWorld(world);
	obstacles = rtt::ai::world::field->getBlockadesMappedToGoal(false, {0, 0});
	visibleParts = rtt::ai::world::field->getVisiblePartsOfGoal(false, {0, 0});
	EXPECT_EQ(obstacles.size(), 1);
	EXPECT_EQ(visibleParts.size(), 2);
	EXPECT_GT(obstacles.at(0).first.dist(obstacles.at(0).second), rtt::ai::Constants::ROBOT_RADIUS()); // the obstacle should be greater than robot radius
}

