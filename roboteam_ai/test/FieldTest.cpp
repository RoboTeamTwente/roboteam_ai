//
// Created by mrlukasbos on 19-10-18.
//

#include <gtest/gtest.h>
#include "../src/utilities/Field.h"

TEST(FieldTest, it_gets_and_sets_the_field) {
    roboteam_msgs::GeometryFieldSize field;
    field.boundary_width = 42;

    rtt::ai::Field::set_field(field);
    ASSERT_EQ(rtt::ai::Field::get_field().boundary_width, 42);
}

TEST(FieldTest, it_gets_points_in_defence_area) {
    rtt::ai::Field::pointIsInDefenceArea(rtt::Vector2(0, 19));
    rtt::ai::Field::get_our_goal_center();
    rtt::ai::Field::get_their_goal_center();

}
