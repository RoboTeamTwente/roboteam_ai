//
// Created by rolf on 05/12/18.
//

#include "../src/control/ControlUtils.h"
#include <gtest/gtest.h>

namespace cr=rtt::ai::control;

TEST(ControlUtils, linedistances) {
    Vector2 A(0, 0), B(0, 2), C(1, 1), D(1, 3), E(0, 4);

    double dist = cr::ControlUtils::distanceToLine(C, A, B);
    EXPECT_DOUBLE_EQ(dist, 1.0);

    double dist2 = cr::ControlUtils::distanceToLine(B, A, C);
    EXPECT_DOUBLE_EQ(dist2, sqrt(2));

    double dist3 = cr::ControlUtils::distanceToLineWithEnds(D, A, B);
    EXPECT_DOUBLE_EQ(dist3, sqrt(2));

    double dist4 = cr::ControlUtils::distanceToLineWithEnds(D, A, E);
    EXPECT_DOUBLE_EQ(dist4, 1.0);
}

TEST(ControlUtils,rotateDirection){
    double ang1=-0.8*M_PI,ang2=0.8*M_PI,ang3=-0.2*M_PI, ang4=0.2*M_PI;
    cr::ControlUtils::rotateDirection(ang3,ang4);
    cr::ControlUtils::rotateDirection(ang2,ang4);
}

TEST(ControlUtils, triangleArea){
    {
        Vector2 A(0, 0), B(0, 2), C(1, 1);
        EXPECT_FLOAT_EQ(cr::ControlUtils::TriangleArea(A, B, C), 1);
    }
    {
        Vector2 A(-1, 0), B(0, 1), C(1, 0);
        EXPECT_FLOAT_EQ(cr::ControlUtils::TriangleArea(A, B, C), 1);
    }
    {
        Vector2 A(0, 0), B(0, 100), C(1, 0);
        EXPECT_FLOAT_EQ(cr::ControlUtils::TriangleArea(A, B, C), 50);
    }
}

TEST(ControlUtils, constrainAngle){
    EXPECT_DOUBLE_EQ(cr::ControlUtils::constrainAngle(2.5 * M_PI), 0.5 * M_PI);
    EXPECT_DOUBLE_EQ(cr::ControlUtils::constrainAngle(-2 * M_PI), 0);
    EXPECT_NEAR(cr::ControlUtils::constrainAngle(200 * M_PI), 0, 0.01);
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
        Vector2 A(-2, -2), B(1, 1), C(2, 2);
        EXPECT_TRUE(cr::ControlUtils::onLineSegment(A, B, C));
    }
}

