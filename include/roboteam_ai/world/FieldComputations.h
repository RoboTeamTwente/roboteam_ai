#ifndef ROBOTEAM_AI_FIELDCOMPUTATIONS_H
#define ROBOTEAM_AI_FIELDCOMPUTATIONS_H

#include <include/roboteam_ai/world/Field.h>
#include <roboteam_utils/Polygon.h>
#include <cmath>
#include "mutex"
#include "roboteam_proto/GeometryFieldSize.pb.h"
#include "roboteam_proto/messages_robocup_ssl_geometry.pb.h"
#include <control/ControlUtils.h>
#include <interface/api/Input.h>
#include "world/World.h"
#include "world/WorldData.h"

namespace rtt::ai {
namespace world {
class WorldData;
}  // namespace world
using namespace rtt::ai::world;

/**
 * The Field Computation class does all kind of computations on the Field based on the Field class variables.
 * @author Created by: Lukas Bos <br>
 *         Documented and refactored by: Haico Dorenbos
 * @since 2019-08-30
 */
class FieldComputations {
    private:
        static constexpr double NEGLIGIBLE_LENGTH = 0.000001; // If a line length is below or equal to this threshold then is it neglected during determining the blockades.

    public:
        /**
         * Determines whether a given point is in our/their defence area
         * @param field The field class which is used to determine the position of the defence areas.
         * @param point The point for which it is checked whether it is in our/their defence area.
         * @param isOurDefenceArea True if it is checked whether the point is in our defence area, false if it is checked whether the point is in the opponents defence area (if
         * this parameter is not set then it checks whether the point is in our defence area).
         * @param margin The outwards margin with which the defence area will be expanded/shrinked for determining whether the point is in our defence area. A positive value means
         * that the rectangular defence area will be expanded in both y directions and towards the center of the field, a negative value means that the rectangular defence area
         * will be shrinked in both y directions and towards the goal of that defence area (if this paremeter is not set then the defence area will not be expanded/shrinked).
         * @param includeOutsideField If set to true then the defence area will also be expanded/shrinked at the goal side (boundary side) of the field, if set to false then it
         * will not be expanded/shrinked at the goal side (boundary side) of the field (if this parameter is not set then the defence area will not be expanded/shrinked at the goal
         * side (boundary side) of the field).
         * @return True if the point is in the given defence area, false if the point is not in the given defence area.
         */
        static bool pointIsInDefenceArea(const Field &field, const Vector2& point, bool isOurDefenceArea = true, double margin = 0.0, bool includeOutsideField = false);

        /**
         * Check whether a given point is in the field.
         * @param field The field class which is used to determine the boundaries of the field.
         * @param point The point for which it is checked whether it is in the field or not.
         * @param margin The outwards margin in which the rectangular field area will get expanded in all directions. A positive value means that the field will be expanded with
         * the absolute margin value in all directions. A negative value means that the field will be shrinked with the absolute margin value in all directions.
         * @return True if the point is in the field, false if the point is not in the field.
         */
        static bool pointIsInField(const Field &field, const Vector2& point, double margin = 0.0);

        /**
         * Get the percentage of goal visible from a given point, i.e. how much of the goal can be reached by directly shoot a ball over the ground from a given point without
         * hitting any robot from a given team.
         * @param field The field used to determine where the goals are.
         * @param ourGoal True if we want to check how much of our goal is visible from the given point. False if we want to check how much of the opponents goal is visible from
         * the given point.
         * @param point The point from which it is checked how much of the goal is visible (how much of the goal can be reached by a direct shoot over the ground).
         * @param world Data about the world which is used to determine the locations of all robots.
         * @param id The id of the robot which is not considered as blockade. Set this value to -1 if you do not want to exclude a particular robot as blockade (when this parameter
         * is not set we do not exclude a particular robot as blockade).
         * @param ourTeam True if all the robots from our team get excluded as blockades. False if all the robots from the opponents team get excluded as blockades (when this
         * parameter is not set then all robots from the opponents team get excluded as blockades).
         * @return The percentage of the goal visible (directly shootable) at that point, which is a double value between 0.0 and 100.0 including both.
         */
        static double getPercentageOfGoalVisibleFromPoint(const Field &field, bool ourGoal, const Vector2& point, const world::WorldData &world, int id = -1, bool ourTeam = false);

        static std::vector<Line> getVisiblePartsOfGoal(const Field &field, bool ourGoal, const Vector2& point, const world::WorldData &world);
        static Line getGoalSides(const Field &field, bool ourGoal);
        static double getDistanceToGoal(const Field &field, bool ourGoal, const Vector2& point);
        static Vector2 getPenaltyPoint(const Field &field, bool ourGoal);
        static std::shared_ptr<Vector2> lineIntersectionWithDefenceArea(const Field &field, bool ourGoal, const Vector2& lineStart, const Vector2& lineEnd,double margin);
        static double getTotalGoalAngle(const Field &field, bool ourGoal, const Vector2& point);
        static Polygon getDefenseArea(const Field &field, bool ourDefenseArea = true, double margin = 0.0, bool includeOutSideField = true);
        static Polygon getGoalArea(const Field &field, bool ourGoal = true, double margin = 0.0, bool hasBackMargin = false);
        static Polygon getFieldEdge(const Field &field, double margin = 0.0);

    private:
        /**
         * Check which part of the goal are blocked by robots, i.e. to which parts of the goal the ball can be shoot over the ground from a given point without hitting any robot
         * from a given team.
         * @param field The field used to determine where the goals are.
         * @param ourGoal True if the blockades on our goal are mapped. False if the blockades on the opponents goals are mapped.
         * @param point The point from which it is checked what places of the goal are blocked (which places cannot be reached by a direct shoot over the ground).
         * @param world Data about the world which is used to determine the locations of all robots.
         * @param id The id of the robot which is not considered as blockade. Set this value to -1 if you do not want to exclude a particular robot as blockade (when this parameter
         * is not set we do not exclude a particular robot as blockade).
         * @param ourTeam True if all the robots from our team get excluded as blockades. False if all the robots from the opponents team get excluded as blockades (when this
         * parameter is not set then all robots from the opponents team get excluded as blockades).
         * @return All the parts of the goal that are blocked (not directly reachable by a direct shoot over the ground).
         */
        static std::vector<LineSegment> getBlockadesMappedToGoal(const Field &field, bool ourGoal, const Vector2& point, const world::WorldData &world, int id = -1,
            bool ourTeam = false);

        /**
         * Check whether a given robot really blocks a part of the goal (which is not the case if the robot belongs to a given team or if the robot has a given id) and if so return
         * the given part of the goal that is blocked by this robot.
         * @param ourGoal True if blockade by the given robot is mapped to our goal. False if the blockade by the given robot is mapped to the opponents goal.
         * @param point The point from which it is checked which places of the goal are blocked by the given robot (which places cannot be reached by a direct shoot over the ground
         * because of the given robot).
         * @param id The id of the robot which is not considered as blockade. This id is checked with the id of the given robot to determine if it really blocks a part of the goal.
         * @param ourTeam True if all the robots from our team get excluded as blockades. False if all the robots from the opponents team get excluded as blockades. So if the given
         * robot is from this team then it is not considered to really block a part of the goal.
         * @param robot The robot of which it is determined whether it really blocks a part of the goal and which part of the goal is blocked by this robot.
         * @param robotRadius The radius of the robot that creates a 'circle' through which the ball is 'considered' not able to pass.
         * @param goalSide The goal side of which is it checked whether it is really blocked by the robot and which parts of this goal side are blocked.
         * @return No LineSegment in case the robot does not block a part of the goal or the robot is considered to be a non-blocking robot (because belongs to the given team or
         * has the given id). Otherwise it returns the blocked part of the goal by this robot.
         */
        static std::optional<LineSegment> robotBlockade(bool ourGoal, const Vector2 &point, int id, bool ourTeam, std::shared_ptr<Robot> robot,
            const double robotRadius, LineSegment goalSide);

        /**
         * Merge overlapping blockade line segments on the goal side into non-overlapping blockade line segments.
         * @param blockades The non-merge possibly overlapping blockade line segments on the goal side.
         * @return Non-overlapping blockade line segments that cover all blocked goal sides.
         */
        static std::vector<LineSegment> mergeBlockades(std::vector<LineSegment> blockades);
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_FIELDCOMPUTATIONS_H
