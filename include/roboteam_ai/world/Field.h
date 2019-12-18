#ifndef RTT_FIELD_H
#define RTT_FIELD_H

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
    /* The width of the field (measured in meters), which is the difference in y-coordinate between the upper part of
     * the field and the lower part of the field. */
    FIELD_WIDTH,
    /* The length of the field (measured in meters), which is the difference in x-coordinate between the left side of
     * the field (where our goal is placed) and the right side of the field (where the opponents goal is placed). */
    FIELD_LENGTH,
    /* The width of both ours and the opponents goal (measured in meters, which is the y-coordinate difference between
     * both goalposts). */
    GOAL_WIDTH,
    // The difference in x-coordinate (measured in meters) between the open part of the goal and the closed part of the goal.
    GOAL_DEPTH,
    BOUNDARY_WIDTH, // The width (measured in meters) of the boundary around the field.
    CENTER_Y, // The center y-coordinate of the field (the y-coordinate that corresponds with the center of the field)
    LEFTMOST_X, //The leftmost x-coordinate of the field (the x-coordinate closest to our goal)
    RIGHTMOST_X, //The rightmost x-coordinate of the field (the x-coordinate closest to the opponents goal)
    BOTTOMMOST_Y, //The bottommost y-coordinate of the field (the y-coordinate corresponding to the bottom side of the field)
    TOPMOST_Y, //The uppermost y-coordinate of the field (the y-coordinate corresponding to the upper side of the field)
};

enum FieldLineName {
    TOP_LINE, // The field line with the highest y-coordinate which goes from the left side to the right side of the field.
    BOTTOM_LINE, // The field line with the lowest y-coordinate which goes from the left side to the right side of the field.
    LEFT_LINE, // The field line left from our goal (our goal is always adjacent to this line).
    RIGHT_LINE, // The field line right from the opponents goal (their goal is always adjacent to this line)
    /* The line that seperates our side from the field and the opponent side of the field (this line moves in the y
     * direction of the field, i.e. the width of the field) */
    HALF_LINE,
    /* The line that start from the middle point of our goal and ends at the middle point of the opponents goal (this
     * line moves in the x direction of the field, i.e. the length of the field) */
    CENTER_LINE,
    LEFT_PENALTY_LINE, // The line parallel to the left line and our goal. This line marks our goal area.
    RIGHT_PENALTY_LINE, // The line parallel to the right line and the opponents goal. This line marks their goal area.
    /* The line closest (of all our goal area lines) to the top line and parallel to the top line. This line marks our
     * goal area. */
    TOP_LEFT_PENALTY_STRETCH,
    /* The line closest (of all our goal area lines) to the bottom line and parallel to the bottom line. This line marks
     * our goal area. */
    BOTTOM_LEFT_PENALTY_STRETCH,
    /* The line closest (of all the opponents goal area lines) to the top line and parallel to the top line. This line
     * marks the opponents goal area. */
    TOP_RIGHT_PENALTY_STRETCH,
    /* The line closest (of all the opponents goal area lines) to the bottom line and parallel to the bottom line. This
     * line marks the opponents goal area. */
    BOTTOM_RIGHT_PENALTY_STRETCH
};

enum FieldVectorName {
    OUR_GOAL_CENTER, // The middle point of our goal (this point is on the left line).
    THEIR_GOAL_CENTER, // The middle point of the opponents goal (this point is on the right line).
    LEFT_PENALTY_POINT, // The penalty point from which penalties are made towards our goal.
    RIGHT_PENALTY_POINT, // The penalty point from which penalties are made towards the opponents goal.
    OUR_BOTTOM_GOAL_SIDE, // The bottom most point of our goal (this point is on the left line).
    OUR_TOP_GOAL_SIDE, // The top most point of our goal (this point is on the left line).
    THEIR_BOTTOM_GOAL_SIDE, // The bottom most point of the opponents goal (this point is on the right line).
    THEIR_TOP_GOAL_SIDE, // The top most point of the opponents goal (this point is on the right line).
};

enum FieldArcName {
    CENTER_CIRCLE // The circle in the middle from which the ball will be kicked off
};

/**
 * Stores all data which is directly obtained by the field camera (this data can change during the match) and store all
 * singular constant data (data that does not change through the match) about the field, which together includes: <br>
 * - Length, widths, heights of the field, goals and boundary. <br>
 * - The location, direction and sizes of all lines on the field. <br>
 * - The location and sizes of all arcs on the field. <br>
 * - Important and frequently used locations of the field, e.g. positions around our and the opponents goal.
 *
 * @author Created by: Lukas Bos <br>
 *         Documented and refactored by: Haico Dorenbos
 * @since 2019-08-30
 */
class Field {
    FRIEND_TEST(FieldTest, line_intersects_with_defence_area);
    FRIEND_TEST(FieldTest, it_gets_points_in_defence_area);
    FRIEND_TEST(FieldTest, it_returns_proper_goal_centers);
    FRIEND_TEST(FieldTest, it_detects_points_in_field_properly);
    FRIEND_TEST(FieldTest, it_calculates_obstacles);
    FRIEND_TEST(FieldTest, penalty_points);
    FRIEND_TEST(FieldTest, goal_angle);

private:
    // Used to convert protobuf names to field names.
    std::unordered_map<std::string, std::string> NAME_MAP = {
        {"TopTouchLine", "top_line"},
        {"BottomTouchLine", "bottom_line"},
        {"LeftGoalLine", "left_line"},
        {"RightGoalLine", "right_line"},
        {"HalfwayLine", "half_line"},
        {"CenterLine", "center_line"},
        {"LeftPenaltyStretch", "left_penalty_line"},
        {"RightPenaltyStretch", "right_penalty_line"},
        {"LeftFieldLeftPenaltyStretch", "top_left_penalty_stretch"},
        {"LeftFieldRightPenaltyStretch", "bottom_left_penalty_stretch"},
        {"RightFieldLeftPenaltyStretch", "bottom_right_penalty_stretch"},
        {"RightFieldRightPenaltyStretch", "top_right_penalty_stretch"},
        {"CenterCircle", "center_circle"},
    };

    // Used to convert field line name, in string format, to the corresponding FieldLineName enum value
    std::unordered_map<std::string, FieldLineName> CONVERT_TO_FIELD_LINE_NAME = {
        {"top_line", TOP_LINE},
        {"bottom_line", BOTTOM_LINE},
        {"left_line", LEFT_LINE},
        {"right_line", RIGHT_LINE},
        {"half_line", HALF_LINE},
        {"center_line", CENTER_LINE},
        {"left_penalty_line", LEFT_PENALTY_LINE},
        {"right_penalty_line", RIGHT_PENALTY_LINE},
        {"top_left_penalty_stretch", TOP_LEFT_PENALTY_STRETCH},
        {"bottom_left_penalty_stretch", BOTTOM_LEFT_PENALTY_STRETCH},
        {"top_right_penalty_stretch", TOP_RIGHT_PENALTY_STRETCH},
        {"bottom_right_penalty_stretch", BOTTOM_RIGHT_PENALTY_STRETCH},
    };

    // Used to convert field arc name, in string format, to the corresponding FieldArcName enum value
    std::unordered_map<std::string, FieldArcName> CONVERT_TO_FIELD_ARC_NAME = {
        {"center_circle", CENTER_CIRCLE},
    };

private:
    static std::mutex fieldMutex; // Prevents situations where the field state is changed and read at the same time.
    static Field field; // Stores the field state as a Singleton variable.

    // Stores all the constant of the field (lengths, widths, positions)
    std::unordered_map<FieldValueName, double> fieldValues = {};
    std::unordered_map<FieldLineName, FieldLineSegment> fieldLines = {}; // Stores all the lines of the field
    std::unordered_map<FieldArcName, FieldArc> fieldArcs = {}; // Stores all the arcs of the field
    std::unordered_map<FieldVectorName, Vector2> fieldVectors = {}; // Stores all positions of the field

public:
    /**
     * Constructor that creates an unitialized Field
     */
    Field() = default;

    /**
     * Constructor that converts a protobuf message into a Field Message object.
     * @param sslFieldSize The corresponding protobuf message.
     */
    Field(proto::SSL_GeometryFieldSize sslFieldSize);

    /**
     * Access the field state (using the Singleton pattern).
     * @return The field state object
     */
    static Field get_field();

    /**
     * Set the field state (using the Singleton pattern).
     * @param field The new field state
     */
    static void set_field(Field _field);

    /**
     * Get a value/constant about the field. All values are measured in SI standard units, so lengths/distances/widths
     * are measured in meters.
     * @param valueName The field value name attribute of which we want to get the value.
     * @return The corresponding field value.
     */
    double operator[](FieldValueName arcName) const;

    /**
     * Get one of the lines of the field.
     * @param lineName The field line name which we want to get.
     * @return The corresponding field line.
     */
    FieldLineSegment operator[](FieldLineName lineName) const;

    /**
     * Get one of the arcs of the field.
     * @param arcName The field arc name which we want to get.
     * @return The corresponding field arc.
     */
     FieldArc operator[](FieldArcName arcName) const;

    /**
     * Get one of the positions of the field.
     * @param vectorName The field vector name of which we want to know the position.
     * @return The corresponding field vector which points to this position.
     */
    Vector2 operator[](FieldVectorName arcName) const;

    /**
     * Get all the lines of the field
     * @return A map which contains all field lines
     */
    std::unordered_map<FieldLineName, FieldLineSegment> getField_lines();

private:
    /**
     * Convert a float measured in millimeters to meters (is needed, because proto message contains values measured in
     * millimeters).
     */
    static float mm_to_m(float scalar);

    /**
     * Convert a vector measured in millimeters to a vector measured in meters.
     */
    static Vector2 mm_to_m(Vector2 vector);

    /**
     * Initialize the field values (this function is only called inside the constructor)
     */
    void initFieldValues(const proto::SSL_GeometryFieldSize &sslFieldSize);

    /**
     * Initialize the field lines (this function is only called inside the constructor)
     */
    void initFieldLines(const proto::SSL_GeometryFieldSize &sslFieldSize);

    /**
     * Initialize the field arcs (this function is only called inside the constructor)
     */
    void initFieldArcs(const proto::SSL_GeometryFieldSize &sslFieldSize);

    /**
     * Initialize the field vectors (this function is only called inside the constructor)
     */
    void initFieldVectors();
};

}
#endif //RTT_FIELD_H
