#ifndef RTT_FIELDMESSAGE_H
#define RTT_FIELDMESSAGE_H

#include "roboteam_proto/FieldLineSegment.pb.h"
#include "roboteam_proto/FieldCircularArc.pb.h"
#include "roboteam_proto/messages_robocup_ssl_geometry.pb.h"
#include <roboteam_utils/Vector2.h>
#include "gtest/gtest_prod.h"

namespace rtt {

struct FieldLineSegment {
    Vector2 begin;
    Vector2 end;
    std::string name;
    float thickness;
};

struct FieldArc {
    Vector2 center;
    float radius;
    float a1; // The start angle, which is also called angle 1.
    float a2; // The end angle, which is also called angle 2.
    std::string name;
    float thickness;
};

enum FieldValueName {
    /* The width of the field, which is the difference in y-coordinate between the upper part of the field and
     * the lower part of the field. */
    FIELD_WIDTH,
    /* The length of the field, which is the difference in x-coordinate between the left side of the field (where
     * our goal is placed) and the right side of the field (where the opponents goal is placed). */
    FIELD_LENGTH,
    GOAL_WIDTH, // The width of both ours and the opponents goal (which is the y-coordinate difference between both goalposts).
    GOAL_DEPTH, // The difference in x-coordinate between the open part of the goal and the closed part of the goal.
    BOUNDARY_WIDTH, // The width of the boundary around the field.
    LEFTMOST_X //The leftmost x-coordinate of the field
};

enum FieldLineName {
    TOP_LINE,
    BOTTOM_LINE,
    LEFT_LINE,
    RIGHT_LINE,
    HALF_LINE,
    CENTER_LINE,
    LEFT_PENALTY_LINE,
    RIGHT_PENALTY_LINE,
    TOP_LEFT_PENALTY_STRETCH,
    BOTTOM_LEFT_PENALTY_STRETCH,
    TOP_RIGHT_PENALTY_STRETCH,
    BOTTOM_RIGHT_PENALTY_STRETCH
};

enum FieldArcName {
    TOP_LEFT_PENALTY_ARC,
    BOTTOM_LEFT_PENALTY_ARC,
    TOP_RIGHT_PENALTY_ARC,
    BOTTOM_RIGHT_PENALTY_ARC,
    CENTER_CIRCLE
};

/**
 * Stores all singular constant data (data that does not change through the match) about the field, which includes: <br>
 * - Length, widths, heights of the field, goals and boundary. <br>
 * - The location, direction and sizes of all lines on the field. <br>
 * - The location and sizes of all arcs on the field.
 * @author Created by: Lukas Bos <br>
 *         Documented by: Haico Dorenbos
 * @since 2019-08-30
 */
class FieldMessage {
    FRIEND_TEST(FieldTest, line_intersects_with_defence_area);
    FRIEND_TEST(FieldTest, it_gets_points_in_defence_area);
    FRIEND_TEST(FieldTest, it_returns_proper_goal_centers);
    FRIEND_TEST(FieldTest, it_detects_points_in_field_properly);
    FRIEND_TEST(FieldTest, it_calculates_obstacles);
    FRIEND_TEST(FieldTest, penalty_points);
    FRIEND_TEST(FieldTest, goal_angle);

private:
    std::map<std::string, std::string> NAME_MAP = {
        std::make_pair("TopTouchLine", "top_line"),
        std::make_pair("BottomTouchLine", "bottom_line"),
        std::make_pair("LeftGoalLine", "left_line"),
        std::make_pair("RightGoalLine", "right_line"),
        std::make_pair("HalfwayLine", "half_line"),
        std::make_pair("CenterLine", "center_line"),
        std::make_pair("LeftPenaltyStretch", "left_penalty_line"),
        std::make_pair("RightPenaltyStretch", "right_penalty_line"),
        std::make_pair("LeftFieldLeftPenaltyArc", "top_left_penalty_arc"),
        std::make_pair("LeftFieldRightPenaltyArc", "bottom_left_penalty_arc"),
        std::make_pair("RightFieldLeftPenaltyArc", "top_right_penalty_arc"),
        std::make_pair("RightFieldRightPenaltyArc", "bottom_right_penalty_arc"),
        std::make_pair("LeftFieldLeftPenaltyStretch", "top_left_penalty_stretch"),
        std::make_pair("LeftFieldRightPenaltyStretch", "bottom_left_penalty_stretch"),
        std::make_pair("RightFieldLeftPenaltyStretch", "bottom_right_penalty_stretch"),
        std::make_pair("RightFieldRightPenaltyStretch", "top_right_penalty_stretch"),
        std::make_pair("CenterCircle", "center_circle"),
    };
    std::map<std::string, FieldLineName> CONVERT_TO_FIELD_LINE_NAME = {
        std::make_pair("top_line", TOP_LINE),
        std::make_pair("bottom_line", BOTTOM_LINE),
        std::make_pair("left_line", LEFT_LINE),
        std::make_pair("right_line", RIGHT_LINE),
        std::make_pair("half_line", HALF_LINE),
        std::make_pair("center_line", CENTER_LINE),
        std::make_pair("left_penalty_line", LEFT_PENALTY_LINE),
        std::make_pair("right_penalty_line", RIGHT_PENALTY_LINE),
        std::make_pair("top_left_penalty_stretch", TOP_LEFT_PENALTY_STRETCH),
        std::make_pair("bottom_left_penalty_stretch", BOTTOM_LEFT_PENALTY_STRETCH),
        std::make_pair("top_right_penalty_stretch", TOP_RIGHT_PENALTY_STRETCH),
        std::make_pair("bottom_right_penalty_stretch", BOTTOM_RIGHT_PENALTY_STRETCH)
    };
    std::map<std::string, FieldArcName> CONVERT_TO_FIELD_ARC_NAME = {
        std::make_pair("top_left_penalty_arc", TOP_LEFT_PENALTY_ARC),
        std::make_pair("bottom_left_penalty_arc", BOTTOM_LEFT_PENALTY_ARC),
        std::make_pair("top_right_penalty_arc", TOP_RIGHT_PENALTY_ARC),
        std::make_pair("bottom_right_penalty_arc", BOTTOM_RIGHT_PENALTY_ARC),
        std::make_pair("center_circle", CENTER_CIRCLE)
    };

private:
    std::map<FieldValueName, double> fieldValues = {};
    std::map<FieldLineName, FieldLineSegment> fieldLines = {};
    std::map<FieldArcName, FieldArc> fieldArcs = {};

    // The field lines and arcs. For easy addressing.
    FieldLineSegment left_line;
    FieldLineSegment right_line;

    // All the field lines and arcs again. For easy iterating.
    std::vector<FieldLineSegment> field_lines;
    std::vector<FieldArc> field_arcs;

public:
    FieldMessage() = default;
    FieldMessage(roboteam_proto::SSL_GeometryFieldSize sslFieldSize);
    void invert();
    float mm_to_m(float scalar);
    Vector2 mm_to_m(Vector2 vector);

    double get(FieldValueName valueName);
    FieldLineSegment get(FieldLineName lineName);
    FieldArc get(FieldArcName arcName);
    std::vector<FieldLineSegment> getField_lines();
    std::vector<FieldArc> getField_arcs();

    FieldLineSegment getLeft_line();
    FieldLineSegment getRight_line();
    FieldLineSegment getLeft_penalty_line();
    FieldLineSegment getRight_penalty_line();
    FieldLineSegment getTop_right_penalty_stretch();
    FieldArc getCenter_circle();

private:
    void invertArc(FieldArc &arc) const;
    void invertFieldLine(FieldLineSegment &line) const;

};

}
#endif //RTT_FIELDMESSAGE_H
