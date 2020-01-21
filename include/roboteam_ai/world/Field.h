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
    std::unordered_map<std::string, std::optional<FieldLineSegment>*> RELATED_FIELD_LINE = {
        {"top_line", &topLine},
        {"bottom_line", &bottomLine},
        {"left_line", &leftLine},
        {"right_line", &rightLine},
        {"half_line", &halfLine},
        {"center_line", &centerLine},
        {"left_penalty_line", &leftPenaltyLine},
        {"right_penalty_line", &rightPenaltyLine},
        {"top_left_penalty_stretch", &topLeftPenaltyStretch},
        {"bottom_left_penalty_stretch", &bottomLeftPenaltyStretch},
        {"top_right_penalty_stretch", &topRightPenaltyStretch},
        {"bottom_right_penalty_stretch", &bottomRightPenaltyStretch}
    };

    // Used to convert field arc name, in string format, to the corresponding FieldArcName enum value
    std::unordered_map<std::string, std::optional<FieldArc>*> RELATED_FIELD_ARC = {
        {"center_circle", &centerCircle},
    };

private:
    std::vector<FieldLineSegment> allFieldLines;

    /* The width of the field (measured in meters), which is the difference in y-coordinate between the upper part of
     * the field and the lower part of the field. */
    std::optional<double> fieldWidth;

    /* The length of the field (measured in meters), which is the difference in x-coordinate between the left side of
     * the field (where our goal is placed) and the right side of the field (where the opponents goal is placed). */
    std::optional<double> fieldLength;

    /* The width of both ours and the opponents goal (measured in meters, which is the y-coordinate difference between
     * both goalposts). */
    std::optional<double> goalWidth;

    // The difference in x-coordinate (measured in meters) between the open part of the goal and the closed part of the goal.
    std::optional<double> goalDepth;

    // The width (measured in meters) of the boundary around the field.
    std::optional<double> boundaryWidth;

    // The center y-coordinate of the field (the y-coordinate that corresponds with the center of the field)
    std::optional<double> centerY;

    // The leftmost x-coordinate of the field (the x-coordinate closest to our goal)
    std::optional<double> leftmostX;

    //The rightmost x-coordinate of the field (the x-coordinate closest to the opponents goal)
    std::optional<double> rightmostX;

    //The bottommost y-coordinate of the field (the y-coordinate corresponding to the bottom side of the field)
    std::optional<double> bottommostY;

    //The uppermost y-coordinate of the field (the y-coordinate corresponding to the upper side of the field)
    std::optional<double> topmostY;

    // The field line with the highest y-coordinate which goes from the left side to the right side of the field.
    std::optional<FieldLineSegment> topLine;

    // The field line with the lowest y-coordinate which goes from the left side to the right side of the field.
    std::optional<FieldLineSegment> bottomLine;

    // The field line left from our goal (our goal is always adjacent to this line).
    std::optional<FieldLineSegment> leftLine;

    // The field line right from the opponents goal (their goal is always adjacent to this line).
    std::optional<FieldLineSegment> rightLine;

    /* The line that seperates our side from the field and the opponent side of the field (this line moves in the y
     * direction of the field, i.e. the width of the field) */
    std::optional<FieldLineSegment> halfLine;

    /* The line that start from the middle point of our goal and ends at the middle point of the opponents goal (this
     * line moves in the x direction of the field, i.e. the length of the field) */
    std::optional<FieldLineSegment> centerLine;

    // The line parallel to the left line and our goal. This line marks our goal area.
    std::optional<FieldLineSegment> leftPenaltyLine;

    // The line parallel to the right line and the opponents goal. This line marks their goal area.
    std::optional<FieldLineSegment> rightPenaltyLine;

    /* The line closest (of all our goal area lines) to the top line and parallel to the top line. This line marks our
     * goal area. */
    std::optional<FieldLineSegment> topLeftPenaltyStretch;

    /* The line closest (of all our goal area lines) to the bottom line and parallel to the bottom line. This line marks
     * our goal area. */
    std::optional<FieldLineSegment> bottomLeftPenaltyStretch;

    /* The line closest (of all the opponents goal area lines) to the top line and parallel to the top line. This line
     * marks the opponents goal area. */
    std::optional<FieldLineSegment> topRightPenaltyStretch;

    /* The line closest (of all the opponents goal area lines) to the bottom line and parallel to the bottom line. This
     * line marks the opponents goal area. */
    std::optional<FieldLineSegment> bottomRightPenaltyStretch;

    // The middle point of our goal (this point is on the left line).
    std::optional<Vector2> ourGoalCenter;

    // The middle point of the opponents goal (this point is on the right line).
    std::optional<Vector2>  theirGoalCenter;

    // The penalty point from which penalties are made towards our goal.
    std::optional<Vector2> leftPenaltyPoint;

    // The penalty point from which penalties are made towards the opponents goal.
    std::optional<Vector2> rightPenaltyPoint;

    // The bottom most point of our goal (this point is on the left line).
    std::optional<Vector2> ourBottomGoalSide;

    // The top most point of our goal (this point is on the left line).
    std::optional<Vector2> ourTopGoalSide;

    // The bottom most point of the opponents goal (this point is on the right line).
    std::optional<Vector2> theirBottomGoalSide;

    // The top most point of the opponents goal (this point is on the right line).
    std::optional<Vector2> theirTopGoalSide;

    // The circle in the middle from which the ball will be kicked off
    std::optional<FieldArc> centerCircle;

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

    // All getters for the different stored field constants.
    double getFieldWidth() const;
    double getFieldLength() const;
    double getGoalWidth() const;
    double getGoalDepth() const;
    double getBoundaryWidth() const;
    double getCenterY() const;
    double getLeftmostX() const;
    double getRightmostX() const;
    double getBottommostY() const;
    double getTopmostY() const;
    const FieldLineSegment &getTopLine() const;
    const FieldLineSegment &getBottomLine() const;
    const FieldLineSegment &getLeftLine() const;
    const FieldLineSegment &getRightLine() const;
    const FieldLineSegment &getHalfLine() const;
    const FieldLineSegment &getCenterLine() const;
    const FieldLineSegment &getLeftPenaltyLine() const;
    const FieldLineSegment &getRightPenaltyLine() const;
    const FieldLineSegment &getTopLeftPenaltyStretch() const;
    const FieldLineSegment &getBottomLeftPenaltyStretch() const;
    const FieldLineSegment &getTopRightPenaltyStretch() const;
    const FieldLineSegment &getBottomRightPenaltyStretch() const;
    const Vector2 &getOurGoalCenter() const;
    const Vector2 &getTheirGoalCenter() const;
    const Vector2 &getLeftPenaltyPoint() const;
    const Vector2 &getRightPenaltyPoint() const;
    const Vector2 &getOurBottomGoalSide() const;
    const Vector2 &getOurTopGoalSide() const;
    const Vector2 &getTheirBottomGoalSide() const;
    const Vector2 &getTheirTopGoalSide() const;
    const FieldArc &getCenterCircle() const;

    /**
     * Get all the lines of the field
     * @return A map which contains all field lines
     */
    const std::vector<FieldLineSegment> &getFieldLines() const;

private:
    /**
     * This method deals with getting field values and what should happen when a field value is missing.
     */
    double getFieldValue(const std::optional<double> &fieldValue) const;

    /**
     * This method deals with getting field lines and what should happen when a field line is missing.
     */
    const FieldLineSegment &getFieldLine(const std::optional<FieldLineSegment> &fieldLine) const;

    /**
     * This method deals with getting field vectors and what should happen when a field vector is missing.
     */
    const Vector2 &getFieldVector(const std::optional<Vector2> &fieldLine) const;

    /**
     * This method deals with getting field arcs and what should happen when a field arc is missing.
     */
    const FieldArc &getFieldArc(const std::optional<FieldArc> &fieldArc) const;

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
