#ifndef RTT_FIELD_H
#define RTT_FIELD_H

#include <proto/messages_robocup_ssl_geometry.pb.h>
#include <roboteam_utils/Grid.h>
#include <roboteam_utils/Vector2.h>

#include <optional>

#include "gtest/gtest_prod.h"

namespace rtt::world {

struct FieldLineSegment {
    Vector2 begin;
    Vector2 end;
    std::string name;
};

struct FieldArc {
    Vector2 center;
    float radius;
    float a1;  // The start angle, which is also called angle 1.
    float a2;  // The end angle, which is also called angle 2.
    std::string name;
};

/**
 * Stores all data which is directly obtained by the field camera (this data can change during the match) and store all
 * singular constant data (data that does not change through the match) about the field, which combined includes: <br>
 * - Length, widths, heights of the field, goals and boundary. <br>
 * - The location, direction and sizes of all lines on the field. <br>
 * - The location and sizes of all arcs on the field. <br>
 * - Important and frequently used locations of the field, e.g. positions around our and the opponents goal.
 *
 * All these values are expressed in meters. Moreover the encoding of the field looks like:
 *
 *                          (upperFieldLine)
 *              (-x,y)_________________________ (x,y)
 *                 |                              |
 *                 |                              |
 * (your side)     |             (0,0)            | (their side)
 * (leftFieldLine) |                              | (rightFieldLine)
 *                 |                              |
 *              (-x, -y)_______________________(x, -y)
 *                          (bottomFieldLine)
 *
 * Thus the left side of the field always corresponds to our side of the field and the right side of the field always corresponds to the opponents side of the field. Moreover
 * every horizontal line of the field stored as FieldLineSegment has their begin position equal to the leftmost position of that line (position with the lowest x-coordinate) and
 * their end position equal to the rightmost position of that line (position with highest x-coordinate). And every vertical line of the field stored as FieldLineSegment has their
 * begin position equal to the bottommost position of that line (position with the lowest y-coordinate) and their end position equal to the topmost position of that line (position
 * with the highest y-coordinate). The top/bottom penalty stretches are an exception for this: they have their end position always inwards the field.
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
    std::unordered_map<std::string, std::optional<FieldLineSegment> *> RELATED_FIELD_LINE = {{"top_line", &topLine},
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
                                                                                             {"bottom_right_penalty_stretch", &bottomRightPenaltyStretch}};

    // Used to convert field arc name, in string format, to the corresponding FieldArcName enum value
    std::unordered_map<std::string, std::optional<FieldArc> *> RELATED_FIELD_ARC = {
        {"center_circle", &centerCircle},
    };

   private:
    // The number of points generated in a grid in the x-direction
    static constexpr int numSegmentsX = 3;

    // The number of points generated in a grid in the y-direction
    static constexpr int numSegmentsY = 3;

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

    // The leftmost x-coordinate of the field which is the lowest x-coordinate value (is a negative value) and is the x-coordinate closest to our goal.
    std::optional<double> leftmostX;

    // The rightmost x-coordinate of the field which is the highest x-coordinate value (is a positive value) and is the x-coordinate closest to the opponents goal.
    std::optional<double> rightmostX;

    // The bottommost y-coordinate of the field which is the lowest y-coordinate value (is a negative value) and is the y-coordinate corresponding to the bottom side of the field.
    std::optional<double> bottommostY;

    // The uppermost y-coordinate of the field which is the highest y-coordinate value (is a positive value) and is the y-coordinate corresponding to the upper side of the field.
    std::optional<double> topmostY;

    // The x-coordinate of the left penalty line (the penalty line closest to our goal).
    std::optional<double> leftPenaltyX;

    // The x-coordinate of the right penalty line (the penalty line closest to the opponents goal).
    std::optional<double> rightPenaltyX;

    /* The highest y-coordinate of the penalty lines (since both penalty lines have the same y-coordinates
     * at top and bottom, we do not need a seperate variable for the left and right penalty line) */
    std::optional<double> penaltyTopY;

    /* The lowest y-coordinate of the penalty lines (since both penalty lines have the same y-coordinates
     * at top and bottom, we do not need a seperate variable for the left and right penalty line) */
    std::optional<double> penaltyBottomY;

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
    std::optional<Vector2> theirGoalCenter;

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

    // The top position, point with the highest y-coordinate, of the left penalty line (our penalty line).
    std::optional<Vector2> leftPenaltyLineTop;

    // The bottom position, point with the lowest y-coordinate, of the left penalty line (our penalty line).
    std::optional<Vector2> leftPenaltyLineBottom;

    // The top position, point with the highest y-coordinate, of the right penalty line (their penalty line).
    std::optional<Vector2> rightPenaltyLineTop;

    // The bottom position, point with the lowest y-coordinate, of the right penalty line (their penalty line).
    std::optional<Vector2> rightPenaltyLineBottom;

    /* The bottom left corner of the field, which is the point on the field with the lowest x-coordinate (is a negative value) and lowest y-coordinate (is a negative value) and is
     * located at our side of the field. */
    std::optional<Vector2> bottomLeftCorner;

    /* The top left corner of the field, which is the point on the field with the lowest x-coordinate (is a negative value) and highest y-coordinate (is a positive value) and is
    located at our side of the field. */
    std::optional<Vector2> topLeftCorner;

    /* The bottom right corner of the field, which is the point on the field with the highest x-coordinate (is a positive value) and lowest y-coordinate (is a negative value) and
     * is located at the opponents side of the field. */
    std::optional<Vector2> bottomRightCorner;

    /* The top right corner of the field, which is the point on the field with the highest x-coordinate (is a positive value) and highest y-coordinate (is a positive value) and is
     * located at the opponents side of the field. */
    std::optional<Vector2> topRightCorner;

    // The circle in the middle from which the ball will be kicked off
    std::optional<FieldArc> centerCircle;

    // The top left corner of our defence area (note that this is not equal to the top of our goal side).
    std::optional<Vector2> topLeftOurDefenceArea;

    // The bottom left corner of our defence area (note that this is not equal to the bottom of our goal side).
    std::optional<Vector2> bottomLeftOurDefenceArea;

    // The top left corner of their defence area (note that this is not equal to the top of their goal side).
    std::optional<Vector2> topRightTheirDefenceArea;

    // The bottom left corner of their defence area (note that this is not equal to the bottom of their goal side).
    std::optional<Vector2> bottomRightTheirDefenceArea;

    // The left area in the back of the field (nearest side to our goal)
    std::optional<Grid> backLeftGrid;

    // The middle area in the back of the field (nearest side to our goal)
    std::optional<Grid> backMidGrid;

    // The right area in the back of the field (nearest side to our goal)
    std::optional<Grid> backRightGrid;

    // The left area in the middle of the field
    std::optional<Grid> middleLeftGrid;

    // The middle area in the middle of the field
    std::optional<Grid> middleMidGrid;

    // The right area in the middle of the field
    std::optional<Grid> middleRightGrid;

    // The left area in the front of the field (nearest side to their goal)
    std::optional<Grid> frontLeftGrid;

    // The middle area in the front of the field (nearest side to their goal)
    std::optional<Grid> frontMidGrid;

    // The right area in the front of the field (nearest side to their goal)
    std::optional<Grid> frontRightGrid;

   public:
    /**
     * Constructor that creates an unitialized Field
     */
    Field() = default;

    /**
     * Copy assignment operator and constructor.
     * Trivial copies, skips the 3 unordered maps.
     * @return
     */
    Field &operator=(Field const &) noexcept;
    Field(Field const &) noexcept;

    /**
     * Move constructors that copy every member except for the first map
     * Please keep in mind that it _does_ allocate.
     */
    Field &operator=(Field &&) noexcept;
    Field(Field &&) noexcept;

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
    double getLeftmostX() const;
    double getRightmostX() const;
    double getBottommostY() const;
    double getTopmostY() const;
    double getLeftPenaltyX() const;
    double getRightPenaltyX() const;
    const FieldLineSegment &getLeftLine() const;
    const FieldLineSegment &getRightLine() const;
    const FieldLineSegment &getLeftPenaltyLine() const;
    const FieldLineSegment &getRightPenaltyLine() const;
    const Vector2 &getOurGoalCenter() const;
    const Vector2 &getTheirGoalCenter() const;
    const Vector2 &getLeftPenaltyPoint() const;
    const Vector2 &getRightPenaltyPoint() const;
    const Vector2 &getOurBottomGoalSide() const;
    const Vector2 &getOurTopGoalSide() const;
    const Vector2 &getTheirBottomGoalSide() const;
    const Vector2 &getTheirTopGoalSide() const;
    const Vector2 &getLeftPenaltyLineTop() const;
    const Vector2 &getLeftPenaltyLineBottom() const;
    const Vector2 &getRightPenaltyLineTop() const;
    const Vector2 &getRightPenaltyLineBottom() const;
    const Vector2 &getBottomLeftCorner() const;
    const Vector2 &getTopLeftCorner() const;
    const Vector2 &getBottomRightCorner() const;
    const Vector2 &getTopRightCorner() const;
    const Vector2 &getTopLeftOurDefenceArea() const;
    const Vector2 &getBottomLeftOurDefenceArea() const;
    const Vector2 &getTopRightTheirDefenceArea() const;
    const Vector2 &getBottomRightTheirDefenceArea() const;
    const FieldArc &getCenterCircle() const;
    const Grid &getBackLeftGrid() const;
    const Grid &getBackMidGrid() const;
    const Grid &getBackRightGrid() const;
    const Grid &getMiddleLeftGrid() const;
    const Grid &getMiddleMidGrid() const;
    const Grid &getMiddleRightGrid() const;
    const Grid &getFrontLeftGrid() const;
    const Grid &getFrontMidGrid() const;
    const Grid &getFrontRightGrid() const;

    /**
     * Get all the lines of the field
     * @return A map which contains all field lines
     */
    const std::vector<FieldLineSegment> &getFieldLines() const;

    /**
     * Only use this function for unit test purposes!
     * @return A field similar to the original field that is used for testing.
     */
    static Field createTestField();

   private:
    /**
     * Only use this function for unit test purposes! Create a field directly by assigning all main field variables.
     */
    Field(double fieldWidth, double fieldLength, double goalWidth, double goalDepth, double boundaryWidth, FieldLineSegment &topLine, FieldLineSegment &bottomLine,
          FieldLineSegment &leftLine, FieldLineSegment &rightLine, FieldLineSegment &halfLine, FieldLineSegment &centerLine, FieldLineSegment &leftPenaltyLine,
          FieldLineSegment &rightPenaltyLine, FieldLineSegment &topLeftPenaltyStretch, FieldLineSegment &bottomLeftPenaltyStretch, FieldLineSegment &topRightPenaltyStretch,
          FieldLineSegment &bottomRightPenaltyStretch, FieldArc &centerCircle);

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
     * This method deals with getting field grids and what should happen when a field grid is missing.
     */
    const Grid &getFieldGrid(const std::optional<Grid> &fieldGrid) const;

    /**
     * Convert a float measured in millimeters to meters (is needed, because proto message contains values measured in
     * millimeters).
     */
    static float mm_to_m(float scalar);

    /**
     * Convert a vector measured in millimeters to a vector measured in meters.
     */
    static Vector2 mm_to_m(const Vector2 &vector);

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
     * Initialize the field grids (this function is only called inse the contructor)
     */
    void initFieldGrids();

    /**
     * Initialize the other field values, linesegments, arcs and vectors (this function is only called inside the constructor)
     */
    void initFieldOthers();
};

}  // namespace rtt::world
#endif  // RTT_FIELD_H
