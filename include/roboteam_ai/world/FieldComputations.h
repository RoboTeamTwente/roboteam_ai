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
         * @param field
         * @param point
         * @param isOurDefenceArea
         * @param margin
         * @param includeOutsideField
         * @return
         */
        static bool pointIsInDefenceArea(const Field &field, const Vector2& point, bool isOurDefenceArea = true, double margin = 0.0, bool includeOutsideField = false);
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
