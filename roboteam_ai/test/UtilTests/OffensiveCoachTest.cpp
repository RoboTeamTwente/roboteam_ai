//
// Created by robzelluf on 3/18/19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/utilities/OffensiveCoach.h>
#include <roboteam_ai/src/utilities/Field.h>
#include <roboteam_ai/src/utilities/World.h>

namespace rtt {
namespace ai {
namespace coach {

TEST(OffensiveCoachTest, calculateNewPositionsTest) {
    roboteam_msgs::GeometryFieldSize field;
    field.field_width = 3;
    field.field_length = 5;
    Field::set_field(field);

    ASSERT_EQ(OffensiveCoach::getOffensivePositions().size(), 0);
    OffensiveCoach::calculateNewPositions();
    ASSERT_GT(OffensiveCoach::getOffensivePositions().size(), 0);
}

}
}
}
