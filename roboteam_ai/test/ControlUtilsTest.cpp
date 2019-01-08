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
