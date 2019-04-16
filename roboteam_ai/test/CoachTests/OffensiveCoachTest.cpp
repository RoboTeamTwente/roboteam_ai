//
// Created by robzelluf on 4/4/19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/coach/OffensiveCoach.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include "../helpers/FieldHelper.h"

TEST(CoachTest, offensive_coach_test) {
    roboteam_msgs::GeometryFieldSize field = testhelpers::FieldHelper::generateField();

    rtt::ai::world::field->set_field(field);

    std::vector<Vector2> offensivePositions = rtt::ai::coach::g_offensiveCoach.getNewOffensivePositions(4);

    int counter = 0;
    std::vector<Vector2> newOffensivePositions;

    // Calculate the next offensive positions 10 times
    while (counter < 10) {
        newOffensivePositions = rtt::ai::coach::g_offensiveCoach.getNewOffensivePositions(3);
        counter++;
    }

    for(int i = 0; i < offensivePositions.size(); i++) {
        ASSERT_NE(offensivePositions.at(i), newOffensivePositions.at(i));
    }
}