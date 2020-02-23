/*
 * FieldComputations
 * This class maintains the FieldGeometry object, which is a message that contains the field geometry
 *  This class also provides helper functions to interface with it.
 */

#ifndef ROBOTEAM_AI_FIELDCOMPUTATIONS_H
#define ROBOTEAM_AI_FIELDCOMPUTATIONS_H

#include "world/Field.h"
#include <roboteam_utils/Polygon.h>
#include <cmath>
#include <include/roboteam_ai/world_new/views/RobotView.hpp>
#include "mutex"
#include "roboteam_proto/GeometryFieldSize.pb.h"
#include "roboteam_proto/messages_robocup_ssl_geometry.pb.h"

namespace rtt::world_new::view {
class WorldDataView;
}

namespace rtt::ai {

using namespace rtt::ai::world;



class FieldComputations {
   public:
    static bool pointIsInDefenceArea(const Field &field, const Vector2 &point, bool isOurDefenceArea = true, double margin = 0.0, bool includeOutsideField = false);
    static bool pointIsInField(const Field &field, const Vector2 &point, double margin = 0.0);  // TODO: Remove margin hack
    static double getPercentageOfGoalVisibleFromPoint(const Field &field, bool ourGoal, const Vector2 &point, world_new::view::WorldDataView &world, int id = -1, bool ourTeam = false);
    static std::vector<Line> getBlockadesMappedToGoal(const Field &field, bool ourGoal, const Vector2 &point, std::vector<world_new::view::RobotView> robots, int id = -1, bool ourTeam = false);
    static std::vector<Line> mergeBlockades(std::vector<Line> blockades);

    static std::vector<Line> getVisiblePartsOfGoal(const Field &field, bool ourGoal, const Vector2 &point, world_new::view::WorldDataView &world);
    static std::vector<Line> getVisiblePartsOfGoalByObstacles(const Field &field, bool ourGoal, const Vector2 &point, const std::vector<world_new::view::RobotView>& robots);

    static Line getGoalSides(const Field &field, bool ourGoal);
    static double getDistanceToGoal(const Field &field, bool ourGoal, const Vector2 &point);
    static Vector2 getPenaltyPoint(const Field &field, bool ourGoal);
    static std::shared_ptr<Vector2> lineIntersectionWithDefenceArea(const Field &field, bool ourGoal, const Vector2 &lineStart, const Vector2 &lineEnd, double margin);
    static double getTotalGoalAngle(const Field &field, bool ourGoal, const Vector2 &point);
    static Polygon getDefenseArea(const Field &field, bool ourDefenseArea = true, double margin = 0.0, bool includeOutSideField = true);
    static Polygon getGoalArea(const Field &field, bool ourGoal = true, double margin = 0.0, bool hasBackMargin = false);
    static Polygon getFieldEdge(const Field &field, double margin = 0.0);
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_FIELDCOMPUTATIONS_H
