//
// Created by rolf on 05/12/18.
//

#include "roboteam_ai/src/control/ControlUtils.h"
#include <gtest/gtest.h>
#include <roboteam_ai/src/world/Field.h>

namespace cr=rtt::ai::control;
using Vector2 = rtt::Vector2;

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
    double ang1 = - 0.8*M_PI, ang2 = 0.8*M_PI, ang3 = - 0.2*M_PI, ang4 = 0.2*M_PI;
    cr::ControlUtils::rotateDirection(ang3, ang4);
    cr::ControlUtils::rotateDirection(ang2, ang4);
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
}

TEST(ControlUtils, accelerationLimiter) {
    rtt::Vector2 testVel;
    double prevTestVel;
    for (int i = 1; i < 200; i ++) {
        testVel = Vector2(5*cos(i/100.0*M_PI), 3*sin(i/30.0*M_PI));
        prevTestVel = Vector2(5*cos((i - 1)/100.0*M_PI), 3*sin((i - 1)/30.0*M_PI)).length();
        EXPECT_LE(cr::ControlUtils::accelerationLimiter(testVel, rtt::ai::Constants::MAX_ACC_UPPER(),
                prevTestVel).length(),
                prevTestVel + rtt::ai::Constants::MAX_ACC_UPPER()/rtt::ai::Constants::TICK_RATE() + 0.01);
        EXPECT_LE(cr::ControlUtils::accelerationLimiter(testVel, rtt::ai::Constants::MAX_ACC_LOWER(),
                prevTestVel).length(),
                prevTestVel + rtt::ai::Constants::MAX_ACC_LOWER()/rtt::ai::Constants::TICK_RATE() + 0.01);
    }
}

TEST(ControlUtils, calculateMaxAcceleration) {
    double upperAcc = rtt::ai::Constants::MAX_ACC_UPPER();
    double lowerAcc = rtt::ai::Constants::MAX_ACC_LOWER();

    EXPECT_DOUBLE_EQ(cr::ControlUtils::calculateMaxAcceleration(Vector2(0.5, 0.0), 0.0), upperAcc);

    EXPECT_DOUBLE_EQ(cr::ControlUtils::calculateMaxAcceleration(Vector2(0.5, 0.0), 0.5*M_PI), lowerAcc);

    EXPECT_DOUBLE_EQ(cr::ControlUtils::calculateMaxAcceleration(Vector2(0.5, 0.0), - 0.5*M_PI), lowerAcc);

    EXPECT_DOUBLE_EQ(cr::ControlUtils::calculateMaxAcceleration(Vector2(0.5, 0.0), - 0.25*M_PI),
            lowerAcc + 0.5*sqrt(2)*(upperAcc-lowerAcc));

    EXPECT_DOUBLE_EQ(cr::ControlUtils::calculateMaxAcceleration(Vector2(0.5, 0.5), 0.0),
            lowerAcc + 0.5*sqrt(2)*(upperAcc-lowerAcc));

    EXPECT_DOUBLE_EQ(cr::ControlUtils::calculateMaxAcceleration(Vector2(0.5, 0.5), 0.5*M_PI),
            lowerAcc + 0.5*sqrt(2)*(upperAcc-lowerAcc));

    EXPECT_DOUBLE_EQ(cr::ControlUtils::calculateMaxAcceleration(Vector2(- 0.5, - 0.5), 0.0),
            lowerAcc + 0.5*sqrt(2)*(upperAcc-lowerAcc));

    EXPECT_DOUBLE_EQ(cr::ControlUtils::calculateMaxAcceleration(Vector2(- 0.5, - 0.5), 0.5*M_PI),
            lowerAcc + 0.5*sqrt(2)*(upperAcc-lowerAcc));

    EXPECT_DOUBLE_EQ(cr::ControlUtils::calculateMaxAcceleration(Vector2(0.5, 0), M_PI/6),
            lowerAcc + 0.5*sqrt(3)*(upperAcc-lowerAcc));

    EXPECT_DOUBLE_EQ(cr::ControlUtils::calculateMaxAcceleration(Vector2(0.5, 0), 5*M_PI/6),
            lowerAcc + 0.5*sqrt(3)*(upperAcc-lowerAcc));

    EXPECT_DOUBLE_EQ(cr::ControlUtils::calculateMaxAcceleration(Vector2(0.5, 0), M_PI/3),
            lowerAcc + 0.5*(upperAcc-lowerAcc));

    EXPECT_DOUBLE_EQ(cr::ControlUtils::calculateMaxAcceleration(Vector2(0.5, 0), 2*M_PI/3),
            lowerAcc + 0.5*(upperAcc-lowerAcc));

    int nTests = 100;
    for (int i = 0; i < nTests; i++) {
        rtt::Angle robotAngle = static_cast<double>(i * 2.0*M_PI / nTests);
        double expectedAcc = lowerAcc + abs(cos(robotAngle)) *(upperAcc-lowerAcc);
        EXPECT_DOUBLE_EQ(cr::ControlUtils::calculateMaxAcceleration(Vector2(0.5, 0), robotAngle), expectedAcc);
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