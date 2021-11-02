//
// Created by Haico Dorenbos on 20-03-2020
//

#include <gtest/gtest.h>
#include <include/roboteam_ai/world/Field.h>
#include <include/roboteam_ai/world/FieldComputations.h>

namespace rtt {
using namespace rtt::world;
using namespace rtt::ai;

double MAXIMUM_DIFFERENCE = 0.000001;

/* Checks whether the margins of the functions that return an Polygon are all in outwards direction by checking if the actual size of the area
 * matches with the expected size of the area. */
TEST(FieldComputationTest, outwards_margin) {
    Field testField = Field::createTestField();

    // Check if the area of the field matches when the field is not changed, when it is expanded and when it is shrinked.
    double expectedFieldSize = testField.getFieldWidth() * testField.getFieldLength();
    double actualFieldSize = FieldComputations::getFieldEdge(testField).area();
    EXPECT_TRUE(fabs(expectedFieldSize - actualFieldSize) < MAXIMUM_DIFFERENCE);
    expectedFieldSize = (testField.getFieldWidth() + 2.0) * (testField.getFieldLength() + 2.0);
    actualFieldSize = FieldComputations::getFieldEdge(testField, 1.0).area();
    EXPECT_TRUE(fabs(expectedFieldSize - actualFieldSize) < MAXIMUM_DIFFERENCE);
    expectedFieldSize = (testField.getFieldWidth() - 2.0) * (testField.getFieldLength() - 2.0);
    actualFieldSize = FieldComputations::getFieldEdge(testField, -1.0).area();
    EXPECT_TRUE(fabs(expectedFieldSize - actualFieldSize) < MAXIMUM_DIFFERENCE);

    // Check if the area of our defence area matches when it is not changed, when it is expanded and when it is shrinked.
    double defenceAreaWidth = testField.getLeftPenaltyX() - testField.getLeftLine().begin.x;
    double defenceAreaHeight = LineSegment(testField.getLeftPenaltyLine().begin, testField.getLeftPenaltyLine().end).length();
    double expectedDefenceAreaSize = defenceAreaWidth * defenceAreaHeight;
    double actualDefenceAreaSize = FieldComputations::getDefenseArea(testField, true, 0.0, 0.0).area();
    EXPECT_TRUE(fabs(expectedDefenceAreaSize - actualDefenceAreaSize) < MAXIMUM_DIFFERENCE);
    expectedDefenceAreaSize = (defenceAreaWidth + 1.5) * (defenceAreaHeight + 2.0);
    actualDefenceAreaSize = FieldComputations::getDefenseArea(testField, true, 1.0, 0.5).area();
    EXPECT_TRUE(fabs(expectedDefenceAreaSize - actualDefenceAreaSize) < MAXIMUM_DIFFERENCE);
    expectedDefenceAreaSize = (defenceAreaWidth - 0.75) * (defenceAreaHeight - 0.5);
    actualDefenceAreaSize = FieldComputations::getDefenseArea(testField, true, -0.25, -0.5).area();
    EXPECT_TRUE(fabs(expectedDefenceAreaSize - actualDefenceAreaSize) < MAXIMUM_DIFFERENCE);

    // Check if the area of the opponents defence area matches when it is not changed, when it is expanded and when it is shrinked.
    defenceAreaWidth = testField.getRightLine().begin.x - testField.getRightPenaltyX();
    defenceAreaHeight = LineSegment(testField.getRightPenaltyLine().begin, testField.getRightPenaltyLine().end).length();
    expectedDefenceAreaSize = defenceAreaWidth * defenceAreaHeight;
    actualDefenceAreaSize = FieldComputations::getDefenseArea(testField, false, 0.0, 0.0).area();
    EXPECT_TRUE(fabs(expectedDefenceAreaSize - actualDefenceAreaSize) < MAXIMUM_DIFFERENCE);
    expectedDefenceAreaSize = (defenceAreaWidth + 1.5) * (defenceAreaHeight + 2.0);
    actualDefenceAreaSize = FieldComputations::getDefenseArea(testField, false, 1.0, 0.5).area();
    EXPECT_TRUE(fabs(expectedDefenceAreaSize - actualDefenceAreaSize) < MAXIMUM_DIFFERENCE);
    expectedDefenceAreaSize = (defenceAreaWidth - 0.75) * (defenceAreaHeight - 0.5);
    actualDefenceAreaSize = FieldComputations::getDefenseArea(testField, false, -0.25, -0.5).area();
    EXPECT_TRUE(fabs(expectedDefenceAreaSize - actualDefenceAreaSize) < MAXIMUM_DIFFERENCE);

    // Check if the area of our goal area matches when it is not changed, when it is expanded and when it is shrinked.
    double goalAreaWidth = testField.getGoalDepth();
    double goalAreaHeight = testField.getGoalWidth();
    double expectedGoalAreaSize = goalAreaWidth * goalAreaHeight;
    double actualGoalAreaSize = FieldComputations::getGoalArea(testField, true, 0.0, false).area();
    EXPECT_TRUE(fabs(expectedGoalAreaSize - actualGoalAreaSize) < MAXIMUM_DIFFERENCE);
    expectedGoalAreaSize = (goalAreaWidth + 0.08) * (goalAreaHeight + 0.08);
    actualGoalAreaSize = FieldComputations::getGoalArea(testField, true, 0.04, true).area();
    EXPECT_TRUE(fabs(expectedGoalAreaSize - actualGoalAreaSize) < MAXIMUM_DIFFERENCE);
    expectedGoalAreaSize = (goalAreaWidth - 0.04) * (goalAreaHeight - 0.08);
    actualGoalAreaSize = FieldComputations::getGoalArea(testField, true, -0.04, false).area();
    EXPECT_TRUE(fabs(expectedGoalAreaSize - actualGoalAreaSize) < MAXIMUM_DIFFERENCE);

    // Check if the area of the opponents goal area matches when it is not changed, when it is expanded and when it is shrinked.
    expectedGoalAreaSize = goalAreaWidth * goalAreaHeight;
    actualGoalAreaSize = FieldComputations::getGoalArea(testField, false, 0.0, false).area();
    EXPECT_TRUE(fabs(expectedGoalAreaSize - actualGoalAreaSize) < MAXIMUM_DIFFERENCE);
    expectedGoalAreaSize = (goalAreaWidth + 0.08) * (goalAreaHeight + 0.08);
    actualGoalAreaSize = FieldComputations::getGoalArea(testField, false, 0.04, true).area();
    EXPECT_TRUE(fabs(expectedGoalAreaSize - actualGoalAreaSize) < MAXIMUM_DIFFERENCE);
    expectedGoalAreaSize = (goalAreaWidth - 0.04) * (goalAreaHeight - 0.08);
    actualGoalAreaSize = FieldComputations::getGoalArea(testField, false, -0.04, false).area();
    EXPECT_TRUE(fabs(expectedGoalAreaSize - actualGoalAreaSize) < MAXIMUM_DIFFERENCE);
}

TEST(FieldComputationTest, point_in_polygon) {
    double smallChanges = 0.001;
    Field testField = Field::createTestField();

    // Test whether the center of the field is indeed located in the field
    Vector2 testPoint = Vector2(0, 0);
    EXPECT_TRUE(FieldComputations::pointIsInField(testField, testPoint, 0.0));

    /* In the following test cases you do boundary value analysis + error guessing by extending the areas and pick 2 corners of this area from which you slightly move inwards and
     * check if these new positions are indeed located in that area. Also you shrink the areas and pick the same corners from which you slightly move outwards and check if these
     * new positions are indeed located outside that area. */
    
    // Do the boundary value analysis + error guessing for the pointInField function
    testPoint = testField.getTopLeftCorner() + Vector2(-1.0, 1.0);
    EXPECT_TRUE(FieldComputations::pointIsInField(testField, testPoint, 1.0));
    testPoint = testField.getTopLeftCorner() + Vector2(-1.0 - smallChanges, 1.0 + smallChanges);
    EXPECT_FALSE(FieldComputations::pointIsInField(testField, testPoint, 1.0));
    testPoint = testField.getBottomRightCorner() + Vector2(-1.0, 1.0);
    EXPECT_TRUE(FieldComputations::pointIsInField(testField, testPoint, -1.0));
    testPoint = testField.getBottomRightCorner() + Vector2(-1.0 + smallChanges, 1.0 - smallChanges);
    EXPECT_FALSE(FieldComputations::pointIsInField(testField, testPoint, -1.0));

    // Do the boundary value analysis + error guessing for our defence area in the pointInDefenseArea function
    testPoint = testField.getLeftPenaltyLineTop() + Vector2(0.3 - smallChanges, 0.3 - smallChanges);
    EXPECT_TRUE(FieldComputations::pointIsInDefenseArea(testField, testPoint, true, 0.3, 0.2));
    testPoint = testField.getLeftPenaltyLineTop() + Vector2(0.3 + smallChanges, 0.3 + smallChanges);
    EXPECT_FALSE(FieldComputations::pointIsInDefenseArea(testField, testPoint, true, 0.3, 0.2));
    testPoint = testField.getBottomLeftOurDefenceArea() + Vector2(0.2 + smallChanges, 0.3 + smallChanges);
    EXPECT_TRUE(FieldComputations::pointIsInDefenseArea(testField, testPoint, true, -0.3, -0.2));
    testPoint = testField.getBottomLeftOurDefenceArea() + Vector2(0.2 - smallChanges, 0.3 - smallChanges);
    EXPECT_FALSE(FieldComputations::pointIsInDefenseArea(testField, testPoint, true, -0.3, -0.2));

    // Do the boundary value analysis + error guessing for their defence area in the pointInDefenseArea function
    testPoint = testField.getRightPenaltyLineTop() + Vector2(0.3 + smallChanges, -0.3 - smallChanges);
    EXPECT_TRUE(FieldComputations::pointIsInDefenseArea(testField, testPoint, false, -0.3, -0.2));
    testPoint = testField.getRightPenaltyLineTop() + Vector2(0.3 - smallChanges, -0.3 + smallChanges);
    EXPECT_FALSE(FieldComputations::pointIsInDefenseArea(testField, testPoint, false, -0.3, -0.2));
    testPoint = testField.getBottomRightTheirDefenceArea() + Vector2(0.2 - smallChanges, -0.3 + smallChanges);
    EXPECT_TRUE(FieldComputations::pointIsInDefenseArea(testField, testPoint, false, 0.3, 0.2));
    testPoint = testField.getBottomRightTheirDefenceArea() + Vector2(0.2 + smallChanges, -0.3 - smallChanges);
    EXPECT_FALSE(FieldComputations::pointIsInDefenseArea(testField, testPoint, false, 0.3, 0.2));
}

TEST(FieldComputationTest, goal_distance) {
    Field testField = Field::createTestField();

    /* Test cases with a point on the goal, point below the goal, point not on the goal but with y-coordinates between both goal
     * endings and point not on the goal with y-coordinates not between both goal endings. */
    Vector2 testPoint = (testField.getOurBottomGoalSide() + testField.getOurTopGoalSide()) / 2;
    double expectedDistance = FieldComputations::getDistanceToGoal(testField, true, testPoint);
    double actualDistance = 0.0;
    EXPECT_TRUE(abs(expectedDistance - actualDistance) < MAXIMUM_DIFFERENCE);
    testPoint = testField.getTheirBottomGoalSide() + Vector2(0, -3);
    expectedDistance = FieldComputations::getDistanceToGoal(testField, false, testPoint);
    actualDistance = 3.0;
    EXPECT_TRUE(abs(expectedDistance - actualDistance) < MAXIMUM_DIFFERENCE);
    testPoint = Vector2(2, 3.6);
    expectedDistance = FieldComputations::getDistanceToGoal(testField, true, testPoint);
    actualDistance = sqrt(73.0);
    EXPECT_TRUE(abs(expectedDistance - actualDistance) < MAXIMUM_DIFFERENCE);
    testPoint = Vector2(4, 0.5);
    expectedDistance = FieldComputations::getDistanceToGoal(testField, false, testPoint);
    actualDistance = 2.0;
    EXPECT_TRUE(abs(expectedDistance - actualDistance) < MAXIMUM_DIFFERENCE);
}

TEST(FieldComputationTest, total_goal_angle) {
    Field testField = Field::createTestField();

    /* Test cases with a point on the goal, point below the goal, point not on the goal with y-coordinates not between both goal
     * endings, point not on the goal with y-coordinates between both goal endings and point that has an angle of 90 degrees at the
     * goal side. */
    Vector2 testPoint = (testField.getTheirBottomGoalSide() + testField.getTheirTopGoalSide()) / 2;
    double expectedAngle = FieldComputations::getTotalGoalAngle(testField, false, testPoint);
    double actualAngle = M_PI;
    EXPECT_TRUE(abs(expectedAngle - actualAngle) < MAXIMUM_DIFFERENCE);
    testPoint = testField.getOurBottomGoalSide() + Vector2(0, -3);
    expectedAngle = FieldComputations::getTotalGoalAngle(testField, true, testPoint);
    actualAngle = 0.0;
    EXPECT_TRUE(abs(expectedAngle - actualAngle) < MAXIMUM_DIFFERENCE);
    testPoint = testField.getTopLeftCorner();
    expectedAngle = FieldComputations::getTotalGoalAngle(testField, false, testPoint);
    actualAngle = atan(5.1 / 12) - atan(3.9 / 12);
    EXPECT_TRUE(abs(expectedAngle - actualAngle) < MAXIMUM_DIFFERENCE);
    testPoint = testField.getTheirTopGoalSide() + Vector2(0, -0.4);
    expectedAngle = FieldComputations::getTotalGoalAngle(testField, true, testPoint);
    actualAngle = atan(0.4 / 12) + atan(0.8 / 12);
    EXPECT_TRUE(abs(expectedAngle - actualAngle) < MAXIMUM_DIFFERENCE);
    testPoint = testField.getTheirBottomGoalSide() + Vector2(1.2, 0);
    expectedAngle = FieldComputations::getTotalGoalAngle(testField, false, testPoint);
    actualAngle = 0.25 * M_PI;
    EXPECT_TRUE(abs(expectedAngle - actualAngle) < MAXIMUM_DIFFERENCE);
}

TEST(FieldComputationTest, line_intersection_with_defence_area) {
    Field testField = Field::createTestField();

    /* Test cases where only an intersection happens with the corner, where a LineSegment is closely parallel to the defence area but
     * does not intersect, where a LineSegment has infinitely many intersections with the boundary, where a LineSegment ends closely
     * before the defence area, where a LineSegment only intersect once with boundary of the defence area and 2 test cases where a
     * LineSegment intersects twice with the boundary of the defence area. */
    Vector2 startTestLine = testField.getLeftPenaltyLineBottom() + Vector2(1.5, 0.5);
    Vector2 endTestLine = testField.getLeftPenaltyLineBottom() + Vector2(-0.5, -1.5);
    std::shared_ptr<Vector2> actualIntersection = FieldComputations::lineIntersectionWithDefenceArea(testField, true, startTestLine, endTestLine, 0.5);
    Vector2 expectedIntersection = testField.getLeftPenaltyLineBottom() + Vector2(0.5, -0.5);
    EXPECT_EQ(*actualIntersection, expectedIntersection);
    startTestLine = testField.getRightPenaltyLineTop() + Vector2(-0.5, -0.499);
    endTestLine = testField.getTopRightTheirDefenceArea() + Vector2(0.5, -0.499);
    actualIntersection = FieldComputations::lineIntersectionWithDefenceArea(testField, false, startTestLine, endTestLine, -0.5);
    EXPECT_EQ(actualIntersection, nullptr);
    startTestLine = testField.getRightPenaltyLineBottom() + Vector2(0.0, 0.5);
    endTestLine = testField.getRightPenaltyLineTop();
    actualIntersection = FieldComputations::lineIntersectionWithDefenceArea(testField, false, startTestLine, endTestLine, 0.0);
    EXPECT_EQ(*actualIntersection, startTestLine);
    startTestLine = testField.getTheirGoalCenter();
    endTestLine = (testField.getLeftPenaltyLineTop() + testField.getLeftPenaltyLineBottom()) / 2;
    actualIntersection = FieldComputations::lineIntersectionWithDefenceArea(testField, true, startTestLine, endTestLine, -0.1);
    EXPECT_EQ(actualIntersection, nullptr);
    endTestLine = testField.getTopLeftOurDefenceArea() * 2 - testField.getBottomLeftOurDefenceArea();
    startTestLine = testField.getLeftPenaltyLineBottom() * 0.99 + endTestLine * 0.01;
    actualIntersection = FieldComputations::lineIntersectionWithDefenceArea(testField, true, startTestLine, endTestLine, 0.0);
    expectedIntersection = (testField.getTopLeftOurDefenceArea() + testField.getLeftPenaltyLineTop()) / 2;
    EXPECT_EQ(*actualIntersection, expectedIntersection);
    startTestLine = testField.getTopRightTheirDefenceArea() + Vector2(-0.5, 0.2);
    endTestLine = testField.getRightPenaltyLineBottom() + Vector2(0.6, -0.2);
    actualIntersection = FieldComputations::lineIntersectionWithDefenceArea(testField, false, startTestLine, endTestLine, 0.2);
    EXPECT_EQ(*actualIntersection, startTestLine);
    Vector2 intersectStart = testField.getBottomLeftOurDefenceArea() * 0.8 + testField.getBottomLeftOurDefenceArea() * 0.2;
    Vector2 intersectEnd = testField.getLeftPenaltyLineBottom() * 0.25 + testField.getLeftPenaltyLineTop() * 0.75;
    startTestLine = intersectStart * 10 - intersectEnd * 9;
    endTestLine = intersectEnd * 10 - intersectStart * 9;
    actualIntersection = FieldComputations::lineIntersectionWithDefenceArea(testField, true, startTestLine, endTestLine, 0.0);
    EXPECT_EQ(*actualIntersection, intersectStart);
}

}  // namespace rtt