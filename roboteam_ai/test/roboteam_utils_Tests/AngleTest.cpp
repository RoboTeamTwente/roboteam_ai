//
// Created by thijs on 3-4-19.
//

#include <gtest/gtest.h>
#include "roboteam_utils/Angle.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

TEST(AngleTest, it_does_angles) {
    Angle angle1 = M_PI * 4;
    EXPECT_DOUBLE_EQ(angle1, 0.0);

    Angle angle2 = M_PI * 5 + 1.0;
    EXPECT_DOUBLE_EQ(angle2, - M_PI + 1.0);

    Angle angle3 = 3;
    EXPECT_DOUBLE_EQ(angle3, 3);
    EXPECT_DOUBLE_EQ(angle3.getAngle(), 3);
    EXPECT_DOUBLE_EQ(angle3.getAngle() + 100, 103);

    Angle angle4 = 4;
    EXPECT_DOUBLE_EQ(angle4, 4 - 2*M_PI);

    Angle angle5 = M_PI;
    EXPECT_DOUBLE_EQ(angle5, -M_PI);

    Angle angle6 = -M_PI;
    EXPECT_DOUBLE_EQ(angle6, -M_PI);

    Angle angle7 = 3.141592;
    EXPECT_DOUBLE_EQ(angle7, 3.141592);

    Angle angle8 = 3.141593;
    EXPECT_DOUBLE_EQ(angle8, 3.141593 - 2*M_PI);

}

TEST(AnglesTest, it_computes_angles) {

    Angle angle1 = 1.0;
    Angle angle2 = 2.0;
    EXPECT_DOUBLE_EQ(angle1 - angle2, -1.0);
    EXPECT_DOUBLE_EQ(angle1 + angle2, 3.0);
    EXPECT_DOUBLE_EQ(angle1 + 2.0, 3.0);
    EXPECT_DOUBLE_EQ(angle1 + 3.0, 4.0 - 2*M_PI);
    EXPECT_DOUBLE_EQ(angle1 + angle2 + angle2, 5.0 - 2*M_PI);

    Angle angle3 = -1.0;
    Angle angle4 = -2.5;
    Angle angle5 = 3.0;
    Angle angle6 = 2.0;

    EXPECT_DOUBLE_EQ(angle3 - angle4, 1.5);
    EXPECT_DOUBLE_EQ(angle4 - angle3, - (angle3 - angle4).getAngle());

    EXPECT_DOUBLE_EQ(angle3 - angle5, -4.0 + 2*M_PI);
    EXPECT_DOUBLE_EQ(angle5 - angle3, - (angle3 - angle5));

    EXPECT_DOUBLE_EQ(angle3 - angle6, -3.0);
    EXPECT_DOUBLE_EQ(angle6 - angle3, - (angle3 - angle6));

    EXPECT_DOUBLE_EQ(angle4 - angle5, -5.5 + 2*M_PI);
    EXPECT_DOUBLE_EQ(angle5 - angle4, - (angle4 - angle5));

    EXPECT_DOUBLE_EQ(angle4 - angle6, -4.5 + 2*M_PI);
    EXPECT_DOUBLE_EQ(angle6 - angle4, - (angle4 - angle6));

    EXPECT_DOUBLE_EQ(angle5 - angle6, 1.0);
    EXPECT_DOUBLE_EQ(angle6 - angle5, -1.0);

    std::vector<Angle> angles = {-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    for (int i = 0; i < angles.size(); i++) {
        for (int j = i+1; j < angles.size(); j++) {
            EXPECT_DOUBLE_EQ(angles[i] - angles[j], - (angles[j] - angles[i]) );
            EXPECT_TRUE(abs(angles[i] - angles[j]) <= M_PI);
        }
    }
    for (auto &angle : angles) {
        Vector2 vec = angle.toVector2(2.0);

        EXPECT_DOUBLE_EQ(vec.x, 2.0*cos(angle));
        EXPECT_DOUBLE_EQ(vec.y, 2.0*sin(angle));
    }

    Vector2 vector8 = {1.0, 1.0};
    Angle angle8 = Angle(vector8);
    EXPECT_DOUBLE_EQ(angle8, M_PI_4);

    Vector2 vector9 = {-1.0, 1.0};
    Angle angle9 = Angle(vector9);
    EXPECT_DOUBLE_EQ(angle9, 3.0*M_PI_4);

    Vector2 vector10 = {1.0, -1.0};
    Angle angle10 = Angle(vector10);
    EXPECT_DOUBLE_EQ(angle10, -M_PI_4);

    Vector2 vector11 = {-1.0, -1.0};
    Angle angle11 = Angle(vector11);
    EXPECT_DOUBLE_EQ(angle11, -3.0*M_PI_4);






}

} // rtt