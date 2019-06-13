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
    field.field_length = 12;
    field.field_width = 8;

    // set the penalty lines
    field.left_penalty_line.begin = rtt::Vector2(- 4, 2);
    field.left_penalty_line.end = rtt::Vector2(- 4, 6);
    field.right_penalty_line.begin = rtt::Vector2(4, 2);
    field.right_penalty_line.end = rtt::Vector2(4, 6);
    field.boundary_width = 2.0;

    // all points should be in our defence area
    rtt::ai::world::field->set_field(field);

    // all points should be in our defence area
    rtt::ai::world::field->set_field(field);

    // all points should be in our defence area
    rtt::ai::world::field->set_field(field);

    // generate 100 random positions in our defence area
    for (int i = 0; i < 100; i ++) {
        auto x = testhelpers::WorldHelper::getRandomValue(- 6, - 4);
        auto y = testhelpers::WorldHelper::getRandomValue(2, 6);

        bool inOurDefenceArea = rtt::ai::world::field->pointIsInDefenceArea(rtt::Vector2(x, y), true, 0.0);

        if (!inOurDefenceArea) {
            std::cout << rtt::Vector2(x, y) << std::endl;
            bool inOurDefenceArea = rtt::ai::world::field->pointIsInDefenceArea(rtt::Vector2(x, y), true, 0.0);

        }
        EXPECT_TRUE(inOurDefenceArea);

        // the points should not be in their defence area
        bool inTheirDefenceArea = rtt::ai::world::field->pointIsInDefenceArea(rtt::Vector2(x, y), false, 0.0);
        EXPECT_FALSE(inTheirDefenceArea);
    }

    // generate 100 random positions outside our defence area ( wrong x value )
    for (int i = 0; i < 100; i ++) {
        auto x = testhelpers::WorldHelper::getRandomValue(- 4, 4);
        auto y = testhelpers::WorldHelper::getRandomValue(2, 6);
        bool inDefenceArea = rtt::ai::world::field->pointIsInDefenceArea(rtt::Vector2(x, y), true, 0.0);
        EXPECT_FALSE(inDefenceArea);
    }


    // generate 100 random positions outside our defence area ( wrong y value )
    for (int i = 0; i < 100; i ++) {
        auto x = testhelpers::WorldHelper::getRandomValue(-6, -4);
        auto y = testhelpers::WorldHelper::getRandomValue(0, 2);
        bool inDefenceArea = rtt::ai::world::field->pointIsInDefenceArea(rtt::Vector2(x, y), true, 0.0);
        EXPECT_FALSE(inDefenceArea);

        y = testhelpers::WorldHelper::getRandomValue(6, 10);
        inDefenceArea = rtt::ai::world::field->pointIsInDefenceArea(rtt::Vector2(x, y), true, 0.0);
        EXPECT_FALSE(inDefenceArea);
    }

    // generate 100 random positions within defense area but outside field
    for (int i = 0; i < 100; i ++) {
        auto x = testhelpers::WorldHelper::getRandomValue(- 8, -6);
        auto y = testhelpers::WorldHelper::getRandomValue(2, 6);

        // it should be fine with includeOutsideField == true
        bool inDefenceArea = rtt::ai::world::field->pointIsInDefenceArea(rtt::Vector2(x, y), true, 0.0, true);
        EXPECT_TRUE(inDefenceArea);

        // otherwise it should fail.
        inDefenceArea = rtt::ai::world::field->pointIsInDefenceArea(rtt::Vector2(x, y), true, 0.0, false);
        EXPECT_FALSE(inDefenceArea);
    }

    // generate 100 random positions within their defense area but outside field
    for (int i = 0; i < 100; i ++) {
        auto x = testhelpers::WorldHelper::getRandomValue(6, 8);
        auto y = testhelpers::WorldHelper::getRandomValue(2, 6);

        // it should be fine with includeOutsideField == true
        bool inDefenceArea = rtt::ai::world::field->pointIsInDefenceArea(rtt::Vector2(x, y), false, 0.0, true);
        EXPECT_TRUE(inDefenceArea);

        // otherwise it should fail.
        inDefenceArea = rtt::ai::world::field->pointIsInDefenceArea(rtt::Vector2(x, y), false, 0.0, false);
        EXPECT_FALSE(inDefenceArea);

    }

}

TEST(FieldTest, it_returns_proper_goal_centers) {
    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 8;
    field.field_width = 12;
    rtt::ai::world::field->set_field(field);

    auto ourGoalCenter = rtt::ai::world::field->get_our_goal_center();
    EXPECT_EQ(ourGoalCenter.x, - 4);
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
    EXPECT_TRUE(
            rtt::ai::world::field->pointIsInField(point, - 0.1)); // expect true if it has a margin outside the field

    // exactly on the edge, negative
    point = {- 4, - 6};
    EXPECT_FALSE(rtt::ai::world::field->pointIsInField(point));
    EXPECT_TRUE(
            rtt::ai::world::field->pointIsInField(point, - 0.1)); // expect true if it has a margin outside the field

    // exactly on the edge, negative
    point = {- 3.8, - 5.8};
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
    auto world = testhelpers::WorldHelper::getWorldMsg(0, 0, true, field);
    robot.pos = rtt::Vector2(0, 0);
    world.us.push_back(robot);
    rtt::ai::world::world->updateWorld(world);
    auto obstacles = rtt::ai::world::field->getBlockadesMappedToGoal(true, {0, 0}, rtt::ai::world::world->getWorld());
    auto visibleParts = rtt::ai::world::field->getVisiblePartsOfGoal(true, {0, 0}, rtt::ai::world::world->getWorld());
    EXPECT_TRUE(obstacles.empty());
    EXPECT_EQ(static_cast<int>(visibleParts.size()), 1);
    EXPECT_FLOAT_EQ(visibleParts.at(0).first.dist(visibleParts.at(0).second), field.goal_width);
    EXPECT_EQ(rtt::ai::world::field->getPercentageOfGoalVisibleFromPoint(true, {0, 0}, rtt::ai::world::world->getWorld()), 100);


    // watch our goal from the center of the field
    // there are no robots in between
    world = testhelpers::WorldHelper::getWorldMsg(0, 0, false, field);
    robot.pos = rtt::Vector2(- 4, 0);
    world.us.push_back(robot);
    rtt::ai::world::world->updateWorld(world);
    obstacles = rtt::ai::world::field->getBlockadesMappedToGoal(true, {0, 0}, rtt::ai::world::world->getWorld());
    visibleParts = rtt::ai::world::field->getVisiblePartsOfGoal(true, {0, 0}, rtt::ai::world::world->getWorld());
    EXPECT_EQ(static_cast<int>(obstacles.size()), 1);
    EXPECT_EQ(static_cast<int>(visibleParts.size()), 2);
    // the width should be somewhere equal to the width of the robot
    EXPECT_NEAR(rtt::ai::world::field->getPercentageOfGoalVisibleFromPoint(true, {0, 0}, rtt::ai::world::world->getWorld()),
            100 - 200*(rtt::ai::Constants::ROBOT_RADIUS() + rtt::ai::Constants::BALL_RADIUS()),1.0);
    EXPECT_NE(obstacles.at(0).first.dist(obstacles.at(0).second), 2*(rtt::ai::Constants::ROBOT_RADIUS()
            + rtt::ai::Constants::BALL_RADIUS())); // the width of the obstacle is twice robot radius
    // watch their goal from the center of the field
    // there are two robots in between, separated
    roboteam_msgs::WorldRobot robot2;
    robot2.id = 1;

    world = testhelpers::WorldHelper::getWorldMsg(0, 0, false, field);
    robot.pos = rtt::Vector2(4, 0);
    robot2.pos = rtt::Vector2(4, 0.3);
    world.us.push_back(robot);
    world.us.push_back(robot2);
    rtt::ai::world::world->updateWorld(world);

    obstacles = rtt::ai::world::field->getBlockadesMappedToGoal(false, {0, 0}, rtt::ai::world::world->getWorld());
    visibleParts = rtt::ai::world::field->getVisiblePartsOfGoal(false, {0, 0}, rtt::ai::world::world->getWorld());
    EXPECT_EQ(static_cast<int>(obstacles.size()), 2);
    EXPECT_EQ(static_cast<int>(visibleParts.size()), 3);
    EXPECT_NEAR(rtt::ai::world::field->getPercentageOfGoalVisibleFromPoint(false, {0, 0}, rtt::ai::world::world->getWorld()),
            100 - 400*(rtt::ai::Constants::ROBOT_RADIUS() + rtt::ai::Constants::BALL_RADIUS()),1.0);

    // watch their goal from the center of the field
    // there are two robots in between, merged
    world = testhelpers::WorldHelper::getWorldMsg(0, 0, false, field);
    robot.pos = rtt::Vector2(4, 0);
    robot2.pos = rtt::Vector2(4, 0.05);
    world.us.push_back(robot);
    world.us.push_back(robot2);
    rtt::ai::world::world->updateWorld(world);
    obstacles = rtt::ai::world::field->getBlockadesMappedToGoal(false, {0, 0}, rtt::ai::world::world->getWorld());
    visibleParts = rtt::ai::world::field->getVisiblePartsOfGoal(false, {0, 0}, rtt::ai::world::world->getWorld());
    EXPECT_EQ(static_cast<int>(obstacles.size()), 1);
    EXPECT_EQ(static_cast<int>(visibleParts.size()), 2);
    EXPECT_GT(obstacles.at(0).first.dist(obstacles.at(0).second),
            rtt::ai::Constants::ROBOT_RADIUS()); // the obstacle should be greater than robot radius
}


TEST(FieldTest, line_intersects_with_defence_area) {
    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 12;
    field.field_width = 8;
    field.goal_width = 1;
    // set the penalty lines
    field.left_penalty_line.begin = rtt::Vector2(-4, -2);
    field.left_penalty_line.end = rtt::Vector2(-4, 2);
    field.right_penalty_line.begin = rtt::Vector2(4, -2);
    field.right_penalty_line.end = rtt::Vector2(4, 2);
    rtt::ai::world::field->set_field(field);
   
    EXPECT_TRUE(rtt::ai::world::field->lineIntersectsWithDefenceArea(true, {0,0}, {-8,0}, 0));
    EXPECT_FALSE(rtt::ai::world::field->lineIntersectsWithDefenceArea(true, {0,0}, {8,0}, 0));

    EXPECT_TRUE(rtt::ai::world::field->lineIntersectsWithDefenceArea(false, {0,0}, {8,0}, 0));
    EXPECT_FALSE(rtt::ai::world::field->lineIntersectsWithDefenceArea(false, {0,0}, {-8,0}, 0));

    EXPECT_FALSE(rtt::ai::world::field->lineIntersectsWithDefenceArea(false, {8,0}, {12,0}, 0));
    EXPECT_FALSE(rtt::ai::world::field->lineIntersectsWithDefenceArea(true, {-8,0}, {-12,0}, 0));

    // exactly on the lines should return true
    EXPECT_TRUE(rtt::ai::world::field->lineIntersectsWithDefenceArea(true, {-4,2}, {-4,6}, 0));
    EXPECT_TRUE(rtt::ai::world::field->lineIntersectsWithDefenceArea(false, {4,2}, {4,6}, 0));

    // margin should make a difference
    // margin for x
    EXPECT_TRUE(rtt::ai::world::field->lineIntersectsWithDefenceArea(true, {-3.9, 0}, {-3.9, 8}, 1.0));
    EXPECT_TRUE(rtt::ai::world::field->lineIntersectsWithDefenceArea(false, {3.9, 0}, {3.9, 8}, 1.0));
    EXPECT_FALSE(rtt::ai::world::field->lineIntersectsWithDefenceArea(true, {-3.9,0}, {-3.9, 8}, 0.0));
    EXPECT_FALSE(rtt::ai::world::field->lineIntersectsWithDefenceArea(false, {3.9,0}, {3.9, 8}, 0.0));

    // margin for y
    EXPECT_TRUE(rtt::ai::world::field->lineIntersectsWithDefenceArea(true, {-8.0, 2.2}, {-3.9, 2.2}, 1.0));
    EXPECT_TRUE(rtt::ai::world::field->lineIntersectsWithDefenceArea(false, {8.0, 2.2}, {3.9, 2.2}, 1.0));
    EXPECT_FALSE(rtt::ai::world::field->lineIntersectsWithDefenceArea(true, {-8,2.2}, {-3.9, 2.2}, 0.0));
    EXPECT_FALSE(rtt::ai::world::field->lineIntersectsWithDefenceArea(false, {8,2.2}, {3.9, 2.2}, 0.0));


    // check if the intersection points are correct
    // when there are multiple intersections, the closest point to lineStart should be returned.
    auto intersection = rtt::ai::world::field->lineIntersectionWithDefenceArea(true, {-3.9, -8}, {-3.9, 8}, 1.0);
    EXPECT_FLOAT_EQ(intersection->x, -3.9);
    EXPECT_FLOAT_EQ(intersection->y, -3.0);

    intersection = rtt::ai::world::field->lineIntersectionWithDefenceArea(true, {-3.9, 8}, {-3.9, -8}, 1.0);
    EXPECT_FLOAT_EQ(intersection->x, -3.9);
    EXPECT_FLOAT_EQ(intersection->y, 3.0);

    intersection = rtt::ai::world::field->lineIntersectionWithDefenceArea(false, {5, -8}, {5, 8}, 0.0);
    EXPECT_FLOAT_EQ(intersection->x, 5);
    EXPECT_FLOAT_EQ(intersection->y, -2);

    intersection = rtt::ai::world::field->lineIntersectionWithDefenceArea(false, {5, 8}, {5, -8}, 0.0);
    EXPECT_FLOAT_EQ(intersection->x, 5);
    EXPECT_FLOAT_EQ(intersection->y, 2);
}

TEST(FieldTest, penalty_points) {
    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 12;
    field.field_width = 8;
    field.goal_width = 1;
    // set the penalty lines
    field.left_penalty_line.begin = rtt::Vector2(-4, -2);
    field.left_penalty_line.end = rtt::Vector2(-4, 2);
    field.right_penalty_line.begin = rtt::Vector2(4, -2);
    field.right_penalty_line.end = rtt::Vector2(4, 2);
    rtt::ai::world::field->set_field(field);

    Vector2 penaltyPointUs = rtt::ai::world::field->getPenaltyPoint(true);
    Vector2 penaltyPointThem = rtt::ai::world::field->getPenaltyPoint(false);

    EXPECT_EQ(penaltyPointUs.x, -4);
    EXPECT_EQ(penaltyPointUs.y, 0);

    EXPECT_EQ(penaltyPointThem.x, 4);
    EXPECT_EQ(penaltyPointThem.y, 0);
}

TEST(FieldTest, goal_angle) {
    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 12;
    field.field_width = 8;
    field.goal_width = 1;
    rtt::ai::world::field->set_field(field);

    EXPECT_FLOAT_EQ(rtt::ai::world::field->getTotalGoalAngle(true, {0,0}), 0.16628246);
    EXPECT_FLOAT_EQ(rtt::ai::world::field->getTotalGoalAngle(false, {0,0}), 0.16628246);
    EXPECT_EQ(rtt::ai::world::field->getTotalGoalAngle(true, {-6,0}), M_PI);
    EXPECT_EQ(rtt::ai::world::field->getTotalGoalAngle(true, {-6,4}), 0);
}
