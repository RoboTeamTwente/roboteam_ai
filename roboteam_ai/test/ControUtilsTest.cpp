//
// Created by rolf on 05/12/18.
//

#include "../src/control/ControlUtils.h"
#include <gtest/gtest.h>

TEST(ControlUtils, linedistances) {
    Vec A(0, 0), B(0, 2), C(1, 1), D(1, 3), E(0, 4);
    double dist = control::ControlUtils::distanceToLine(C, A, B);
    EXPECT_DOUBLE_EQ(dist, 1.0);
    double dist2 = control::ControlUtils::distanceToLine(B, A, C);
    EXPECT_DOUBLE_EQ(dist2, sqrt(2));

    double dist3 = control::ControlUtils::distanceToLineWithEnds(D, A, B);
    EXPECT_DOUBLE_EQ(dist3, sqrt(2));

    double dist4 = control::ControlUtils::distanceToLineWithEnds(D, A, E);
    EXPECT_DOUBLE_EQ(dist4, 1.0);
}
