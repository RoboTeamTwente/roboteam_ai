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