#ifndef ROBOTEAM_AI_FIELDCOMPUTATIONS_H
#define ROBOTEAM_AI_FIELDCOMPUTATIONS_H

#include <include/roboteam_ai/world/Field.h>
#include <roboteam_utils/Polygon.h>
#include <cmath>
#include "mutex"
#include "roboteam_proto/GeometryFieldSize.pb.h"
#include "roboteam_proto/messages_robocup_ssl_geometry.pb.h"

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


        static double getPercentageOfGoalVisibleFromPoint(const Field &field, bool ourGoal, const Vector2& point, const world::WorldData &world, int id = -1, bool ourTeam = false);
        static std::vector<Line> getBlockadesMappedToGoal(const Field &field, bool ourGoal, const Vector2& point, const world::WorldData &world, int id = -1, bool ourTeam = false);
        static std::vector<Line> mergeBlockades(std::vector<Line> blockades);
        static std::vector<Line> getVisiblePartsOfGoal(const Field &field, bool ourGoal, const Vector2& point, const world::WorldData &world);
        static Line getGoalSides(const Field &field, bool ourGoal);
        static double getDistanceToGoal(const Field &field, bool ourGoal, const Vector2& point);
        static Vector2 getPenaltyPoint(const Field &field, bool ourGoal);
        static std::shared_ptr<Vector2> lineIntersectionWithDefenceArea(const Field &field, bool ourGoal, const Vector2& lineStart, const Vector2& lineEnd,double margin);
        static double getTotalGoalAngle(const Field &field, bool ourGoal, const Vector2& point);
        static Polygon getDefenseArea(const Field &field, bool ourDefenseArea = true, double margin = 0.0, bool includeOutSideField = true);
        static Polygon getGoalArea(const Field &field, bool ourGoal = true, double margin = 0.0, bool hasBackMargin = false);
        static Polygon getFieldEdge(const Field &field, double margin = 0.0);
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_FIELDCOMPUTATIONS_H
