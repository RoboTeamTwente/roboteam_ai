//
// Created by rolf on 05/12/18.
//

#include "roboteam_ai/src/control/ControlUtils.h"
#include "roboteam_ai/src/utilities/Constants.h"
#include <gtest/gtest.h>
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/test/helpers/FieldHelper.h>
#include <roboteam_ai/test/helpers/WorldHelper.h>

namespace cr=rtt::ai::control;
using Vector2 = rtt::Vector2;
using Constants = rtt::ai::Constants;
using ControlUtils = rtt::ai::control::ControlUtils;

TEST(ControlUtils, linedistances) {
    rtt::Vector2 A(0, 0), B(0, 2), C(1, 1), D(1, 3), E(0, 4);

    double dist = cr::ControlUtils::distanceToLine(C, A, B);
    EXPECT_DOUBLE_EQ(dist, 1.0);

    double dist2 = cr::ControlUtils::distanceToLine(B, A, C);
    EXPECT_DOUBLE_EQ(dist2, sqrt(2));

    double dist3 = cr::ControlUtils::distanceToLineWithEnds(D, A, B);
    EXPECT_DOUBLE_EQ(dist3, sqrt(2));

    double dist4 = cr::ControlUtils::distanceToLineWithEnds(D, A, E);
    EXPECT_DOUBLE_EQ(dist4, 1.0);
}

TEST(ControlUtils, rotateDirection) {
    //TODO: This is not used? what does this test even do?
    //double ang1 = - 0.8*M_PI;
    double ang2 = 0.8*M_PI;
    double ang3 = - 0.2*M_PI;
    double ang4 = 0.2*M_PI;
    cr::ControlUtils::rotateDirection(ang3, ang4);
    cr::ControlUtils::rotateDirection(ang2, ang4);
}


TEST(ControlUtils, point_in_rectangle) {
    // test a valid rectangle
    std::vector<Vector2> rectangle = {{0,0},{1,0},{1,1}, {0,1}};
    EXPECT_TRUE(cr::ControlUtils::pointInRectangle({0.2, 0.2}, rectangle));
    EXPECT_TRUE(cr::ControlUtils::pointInRectangle({0.8, 0.2}, rectangle));
    EXPECT_TRUE(cr::ControlUtils::pointInRectangle({0.999, 0.001}, rectangle));
    EXPECT_FALSE(cr::ControlUtils::pointInRectangle({1.8, 1.2}, rectangle));
    EXPECT_FALSE(cr::ControlUtils::pointInRectangle({-0.001, 0.2}, rectangle));
    EXPECT_FALSE(cr::ControlUtils::pointInRectangle({-8, 1.2}, rectangle));

    // test an invalid rectangle
    std::vector<Vector2> InvalidRectangle = {{0,0},{1,0},{1,1}};
    EXPECT_FALSE(cr::ControlUtils::pointInRectangle({0.2, 0.2}, InvalidRectangle));
    EXPECT_FALSE(cr::ControlUtils::pointInRectangle({0.8, 0.2}, InvalidRectangle));
    EXPECT_FALSE(cr::ControlUtils::pointInRectangle({0.999, 0.001}, InvalidRectangle));

    // test a rectangle with negative coordinates
    rectangle = {{0,0},{-1,0},{-1,-1}, {0,-1}};
    EXPECT_TRUE(cr::ControlUtils::pointInRectangle({-0.2, -0.2}, rectangle));
    EXPECT_TRUE(cr::ControlUtils::pointInRectangle({-0.8, -0.2}, rectangle));
    EXPECT_TRUE(cr::ControlUtils::pointInRectangle({-0.999, -0.001}, rectangle));
    EXPECT_FALSE(cr::ControlUtils::pointInRectangle({-1.8, -1.2}, rectangle));
    EXPECT_FALSE(cr::ControlUtils::pointInRectangle({-0.001, 0.2}, rectangle));
    EXPECT_FALSE(cr::ControlUtils::pointInRectangle({-8, 1.2}, rectangle));
}

TEST(ControlUtils, angleDifference) {
    EXPECT_FLOAT_EQ(cr::ControlUtils::angleDifference(0.4, 0.5), 0.1);
    EXPECT_FLOAT_EQ(cr::ControlUtils::angleDifference(- 0.4, 0.5), 0.9);
    EXPECT_FLOAT_EQ(cr::ControlUtils::angleDifference(- 0.4*M_PI, - 2*M_PI), 0.4*M_PI);
    EXPECT_FLOAT_EQ(cr::ControlUtils::angleDifference(- 0.4*M_PI, 1*M_PI), 0.6*M_PI);
}

TEST(ControlUtils, velocityLimiter) {
    for (int i = 0; i < 200; i ++) {
        EXPECT_LE(cr::ControlUtils::velocityLimiter(Vector2(- 102 + i*10, 512 + i*8)).length(),
                rtt::ai::Constants::MAX_VEL() + 0.01);
    }

    // check the min velocity as well
    // limit values between 56 and 3000
    for (int i = 0; i < 200; i ++) {
        auto limitedVel = cr::ControlUtils::velocityLimiter(Vector2(- 102 + i*10, 512 + i*8), 3000, 56, false).length();
        EXPECT_GE(limitedVel, 56 - 0.01);
        EXPECT_LE(limitedVel, 3000 + 0.01);
    }

    for (int i = 0; i < 200; i ++) {
        auto limitedVel = cr::ControlUtils::velocityLimiter(Vector2(- 102 + i*10, 512 + i*8), 3000, 56, true).length();
        EXPECT_GE(limitedVel, 0 - 0.01);
        EXPECT_LE(limitedVel, 8 + 0.01);
    }
}

TEST(ControlUtils, accelerationLimiter) {

    Vector2 prevVel;
    Vector2 targetVel;
    Angle targetAngle;
    const double sA = Constants::MAX_ACC_LOWER() / Constants::TICK_RATE();
    const double fA = Constants::MAX_ACC_UPPER() / Constants::TICK_RATE();
    const double sD = Constants::MAX_DEC_LOWER() / Constants::TICK_RATE();
    const double fD = Constants::MAX_DEC_UPPER() / Constants::TICK_RATE();
    const double error = 0.0001;

    prevVel = Vector2(1.0, 0.0);
    targetVel = Vector2(2.0, 0.0);
    EXPECT_NEAR((ControlUtils::accelerationLimiter(targetVel, prevVel, 0.0) - prevVel).length(), fA, error);
    EXPECT_NEAR((ControlUtils::accelerationLimiter(targetVel, prevVel, M_PI_2) - prevVel).length(), sA, error);
    EXPECT_NEAR((ControlUtils::accelerationLimiter(targetVel, prevVel, M_PI) - prevVel).length(), fA, error);
    EXPECT_NEAR((ControlUtils::accelerationLimiter(targetVel, prevVel, M_PI_2*3.0) - prevVel).length(), sA, error);

    targetVel = Vector2(0.0, 0.0);
    EXPECT_NEAR((ControlUtils::accelerationLimiter(targetVel, prevVel, 0.0) - prevVel).length(), fD, error);
    EXPECT_NEAR((ControlUtils::accelerationLimiter(targetVel, prevVel, M_PI_2) - prevVel).length(), sD, error);
    EXPECT_NEAR((ControlUtils::accelerationLimiter(targetVel, prevVel, M_PI) - prevVel).length(), fD, error);
    EXPECT_NEAR((ControlUtils::accelerationLimiter(targetVel, prevVel, M_PI_2*3.0) - prevVel).length(), sD, error);

    targetVel = Vector2(1.01, 0.01);
    EXPECT_EQ(cr::ControlUtils::accelerationLimiter(targetVel, prevVel, 0.0), targetVel);
    prevVel = Vector2(-1.0, 0.8);
    targetVel = Vector2(-0.99, 0.81);
    EXPECT_EQ(ControlUtils::accelerationLimiter(targetVel, prevVel, 0.0), targetVel);
    targetVel = Vector2(-1.01, 0.80);
    EXPECT_EQ(ControlUtils::accelerationLimiter(targetVel, prevVel, 0.0), targetVel);

    double maxAcceleration = std::max(sA, std::max(fA, std::max(sD, fD)));
    double acceleration;
    for (int i = 0; i < 20; i++) {
        for (int j = 0; j < 50; j++) {
            targetVel = {-0.01*i + 0.14*j - 12.0, 0.2*i - 0.02*j +1.0};
            prevVel = {-0.01*i + 0.14*j - 12.0, 0.2*i - 0.02*j +1.0};
            targetAngle = 0.004*i + 0.12*j - 2.0;
            acceleration = (ControlUtils::accelerationLimiter(targetVel, prevVel, targetAngle) - prevVel).length();
            EXPECT_TRUE(acceleration <= maxAcceleration);
        }
    }

}

TEST(ControlUtils, triangleArea) {
    {
        Vector2 A(0, 0), B(0, 2), C(1, 1);
        EXPECT_FLOAT_EQ(cr::ControlUtils::TriangleArea(A, B, C), 1);
    }
    {
        Vector2 A(- 1, 0), B(0, 1), C(1, 0);
        EXPECT_FLOAT_EQ(cr::ControlUtils::TriangleArea(A, B, C), 1);
    }
    {
        Vector2 A(0, 0), B(0, 100), C(1, 0);
        EXPECT_FLOAT_EQ(cr::ControlUtils::TriangleArea(A, B, C), 50);
    }
}

TEST(ControlUtils, constrainAngle) {
    EXPECT_DOUBLE_EQ(cr::ControlUtils::constrainAngle(2.5*M_PI), 0.5*M_PI);
    EXPECT_DOUBLE_EQ(cr::ControlUtils::constrainAngle(- 2*M_PI), 0);
    EXPECT_NEAR(cr::ControlUtils::constrainAngle(200*M_PI), 0, 0.01);
}

// remember the function assumes three colinear points
TEST(ControlUtils, onLineSegment) {
    {
        Vector2 A(0, 0), B(0, 100), C(1, 0);
        EXPECT_FALSE(cr::ControlUtils::onLineSegment(A, B, C));
    }
    {
        Vector2 A(0, 0), B(0, 1), C(0, 2);
        EXPECT_TRUE(cr::ControlUtils::onLineSegment(A, B, C));
    }
    {
        Vector2 A(0, 0), B(1, 1), C(2, 2);
        EXPECT_TRUE(cr::ControlUtils::onLineSegment(A, B, C));
    }
    {
        Vector2 A(- 2, - 2), B(1, 1), C(2, 2);
        EXPECT_TRUE(cr::ControlUtils::onLineSegment(A, B, C));
    }
}

TEST(ControlUtils, lineOrientation) {
    {
        Vector2 A(0, 0), B(0, 100), C(1, 0);
        EXPECT_EQ(cr::ControlUtils::lineOrientation(A, B, C), 1);
    }
    {
        Vector2 A(0, 0), B(0, 1), C(0, 2);
        EXPECT_EQ(cr::ControlUtils::lineOrientation(A, B, C), 0);
    }
    {
        Vector2 A(0, 0), B(1, - 1), C(2, 2);
        EXPECT_EQ(cr::ControlUtils::lineOrientation(A, B, C), 2);
    }
    {
        Vector2 A(- 2, - 2), B(1, 1), C(2, 2);
        EXPECT_EQ(cr::ControlUtils::lineOrientation(A, B, C), 0);
    }
}

TEST(ControlUtils, lineSegmentsIntersect) {
    {
        Vector2 A(0, 0), B(1, 1), C(1, 0), D(0, 1); // cross
        EXPECT_TRUE(cr::ControlUtils::lineSegmentsIntersect(A, B, C, D));
    }
    {
        Vector2 A(0, 1), B(1, 1), C(1, 0), D(1, 1); // touch at corner
        EXPECT_TRUE(cr::ControlUtils::lineSegmentsIntersect(A, B, C, D));
    }
    {
        Vector2 A(- 100, - 2), B(2500, 0), C(- 10, - 1000), D(- 1, 100); // cross
        EXPECT_TRUE(cr::ControlUtils::lineSegmentsIntersect(A, B, C, D));
    }
    {
        Vector2 A(0, 0), B(0, 1), C(0, 2), D(0, 4); // colinear and not touching
        EXPECT_FALSE(cr::ControlUtils::lineSegmentsIntersect(A, B, C, D));
    }
    {
        Vector2 A(0, 0), B(0, 1), C(0, 0.5), D(0, 4); // colinear and touching
        EXPECT_TRUE(cr::ControlUtils::lineSegmentsIntersect(A, B, C, D));
    }

}

TEST(ControlUtils, line_intersection) {
    { // cross
        Vector2 A(0, 0), B(2.4, 2.4), C(2.4, 0), D(0, 2.4);
        Vector2 intersection = cr::ControlUtils::twoLineIntersection(A, B, C, D);
        EXPECT_FLOAT_EQ(intersection.x, 1.2);
        EXPECT_FLOAT_EQ(intersection.y, 1.2);
    }

    { // cross with negative numbers
        Vector2 A(- 1, - 1), B(1, 1), C(- 1, 0), D(0, - 1);
        Vector2 intersection = cr::ControlUtils::twoLineIntersection(A, B, C, D);
        EXPECT_EQ(intersection.x, - 0.5);
        EXPECT_EQ(intersection.y, - 0.5);
    }

    { // the segments are not touching but the lines can be made infintely long
        Vector2 A(0, 1), B(0, 3), C(- 1, 5), D(1, 5);
        Vector2 intersection = cr::ControlUtils::twoLineIntersection(A, B, C, D);
        EXPECT_EQ(intersection.x, 0);
        EXPECT_EQ(intersection.y, 5);
    }

    { // two parallel lines should return the empty vector
        Vector2 A(0, 1), B(0, 3), C(- 1, 1), D(- 1, 5);
        Vector2 intersection = cr::ControlUtils::twoLineIntersection(A, B, C, D);
        EXPECT_EQ(intersection.x, 0);
        EXPECT_EQ(intersection.y, 0);
    }
}

TEST(ControlUtils, project_to_position_within_field) {
    roboteam_msgs::GeometryFieldSize field;
    field.field_width = 20;
    field.field_length = 10;
    rtt::ai::world::field->set_field(field);
    { // the middle point should always be within the field.
        Vector2 pos = cr::ControlUtils::projectPositionToWithinField(Vector2(0, 0));
        EXPECT_FLOAT_EQ(pos.x, 0);
        EXPECT_FLOAT_EQ(pos.y, 0);
    }
    { // another point within the field
        Vector2 pos = cr::ControlUtils::projectPositionToWithinField(Vector2(4, 4.2), 0.5);
        EXPECT_FLOAT_EQ(pos.x, 4);
        EXPECT_FLOAT_EQ(pos.y, 4.2);
    }

    { // the middle point should always be within the field.
        Vector2 pos = cr::ControlUtils::projectPositionToWithinField(Vector2(9, 11.2), 0.5);
        EXPECT_FLOAT_EQ(pos.x, 4.5);
        EXPECT_FLOAT_EQ(pos.y, 9.5);
    }

    { // the middle point should always be within the field.
        Vector2 pos = cr::ControlUtils::projectPositionToWithinField(Vector2(- 90, - 11.2));
        // if you change the margin, check other places and move it to constants. do not just edit this test.
        EXPECT_FLOAT_EQ(pos.x, - 4.8);
        EXPECT_FLOAT_EQ(pos.y, - 9.8);
    }
}

// the function should return weight/distance^2 * normalized vector IF within minDistance, otherwise 0.
TEST(ControlUtils, it_calculates_forces) {
    Vector2 force = {0, 0};

    // distance should be ok. 2/2^2
    force = cr::ControlUtils::calculateForce(Vector2(0, - 2), 2, 3);
    EXPECT_DOUBLE_EQ(force.x, 0);
    EXPECT_DOUBLE_EQ(force.y, - 0.5);

    // distance should be ok. 2/2^2
    force = cr::ControlUtils::calculateForce(Vector2(0, 2), 2, 3);
    EXPECT_DOUBLE_EQ(force.x, 0);
    EXPECT_DOUBLE_EQ(force.y, 0.5);

    // distance not ok.
    force = cr::ControlUtils::calculateForce(Vector2(0, 3.1), 2, 3);
    EXPECT_DOUBLE_EQ(force.x, 0);
    EXPECT_DOUBLE_EQ(force.y, 0);

    // distance ok. this is a negative force because of negative weight. -2/1^2
    force = cr::ControlUtils::calculateForce(Vector2(0, 1), - 2, 3);
    EXPECT_DOUBLE_EQ(force.x, 0);
    EXPECT_DOUBLE_EQ(force.y, - 2);
}


TEST(ControlUtils, object_velocity_aimed_at_point){
    // With a velocity of 0,0 it should always return false
    EXPECT_FALSE(cr::ControlUtils::objectVelocityAimedToPoint({0,0}, {0,0}, {1,0}, 0.3));
    EXPECT_FALSE(cr::ControlUtils::objectVelocityAimedToPoint({0,0}, {0,0}, {-1,0}, 0.3));

    // velocity in wrong direction
    EXPECT_FALSE(cr::ControlUtils::objectVelocityAimedToPoint({0,0}, {1,1}, {1,0}, 0.3));
    EXPECT_FALSE(cr::ControlUtils::objectVelocityAimedToPoint({0,0}, {-1,-1}, {-1,0}, 0.3));

    // velocity in same 'wrong' direction but with loose margins, so it should be true
    EXPECT_TRUE(cr::ControlUtils::objectVelocityAimedToPoint({0,0}, {1,1}, {1,0}, rtt::toRadians(91)));
    EXPECT_TRUE(cr::ControlUtils::objectVelocityAimedToPoint({0,0}, {-1,-1}, {-1,0}, rtt::toRadians(91)));

    // the margin should be pretty strict
    EXPECT_FALSE(cr::ControlUtils::objectVelocityAimedToPoint({0,0}, {1,1}, {1,0}, rtt::toRadians(90)));
    EXPECT_FALSE(cr::ControlUtils::objectVelocityAimedToPoint({0,0}, {-1,-1}, {-1,0}, rtt::toRadians(90)));
}


TEST(ControlUtils, forward_line_intersection){
    Vector2 A(0,0),B(0,3),C(-1,1),D(1,1),E(0,0.5);
    EXPECT_EQ(cr::ControlUtils::twoLineForwardIntersection(A,B,C,D),1.0/3.0);
    EXPECT_EQ(cr::ControlUtils::twoLineForwardIntersection(B,A,C,D),2.0/3.0);
    EXPECT_EQ(cr::ControlUtils::twoLineForwardIntersection(A,E,C,D),2.0);
    EXPECT_EQ(cr::ControlUtils::twoLineForwardIntersection(E,A,C,D),-1.0);
}

TEST(ControlUtils, getInterceptPointOnLegalPosition){

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

    for (int i = 0; i < 500; i++) {

        Vector2 robotpos =  testhelpers::WorldHelper::getRandomFieldPosition(rtt::ai::world::field->get_field());

        auto randomX = testhelpers::WorldHelper::getRandomValue(-2, 2);
        auto randomY = testhelpers::WorldHelper::getRandomValue(-2, 2);
        Vector2 lineStart = {randomX, randomY};

        auto lineEnd = testhelpers::WorldHelper::getRandomFieldPosition(rtt::ai::world::field->get_field());
        rtt::Line line = {lineStart, lineEnd};

        auto newPoint = cr::ControlUtils::getInterceptPointOnLegalPosition(robotpos, line, false, false, 0, 0.1);
        EXPECT_TRUE(rtt::ai::world::field->pointIsInField(newPoint, 0));
        EXPECT_FALSE(rtt::ai::world::field->pointIsInDefenceArea(newPoint, true, -0.01, false));
        EXPECT_FALSE(rtt::ai::world::field->pointIsInDefenceArea(newPoint, false, -0.01, false));
    }
}

