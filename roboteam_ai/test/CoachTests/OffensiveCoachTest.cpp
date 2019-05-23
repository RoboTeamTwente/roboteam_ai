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

    rtt::ai::coach::g_offensiveCoach.updateOffensivePositions();
    std::vector<Vector2> offensivePositions = rtt::ai::coach::g_offensiveCoach.getOffensivePositions(4);
    std::vector<Vector2> newOffensivePositions;

    // Calculate the next offensive positions 100 times
    for(int i = 0; i < 100; i++) {
        rtt::ai::coach::g_offensiveCoach.updateOffensivePositions();
    }
    newOffensivePositions = rtt::ai::coach::g_offensiveCoach.getOffensivePositions(4);

    for(unsigned int i = 0; i < newOffensivePositions.size(); i++) {
        ASSERT_NE(offensivePositions.at(i), newOffensivePositions.at(i));
    }
}