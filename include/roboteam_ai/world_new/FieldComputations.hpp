//
// Created by timovdk on 2/19/20.
//

#ifndef RTT_FIELDCOMPUTATIONS_HPP
#define RTT_FIELDCOMPUTATIONS_HPP

#include <roboteam_utils/Polygon.h>
#include <cmath>
#include "mutex"
#include "roboteam_proto/GeometryFieldSize.pb.h"
#include "roboteam_proto/messages_robocup_ssl_geometry.pb.h"
#include "world/Field.h"
#include "world_new/World.hpp"

/**
 * FieldComputations
 * This class maintains the FieldGeometry object, which is a message that contains the field geometry
 *  This class also provides helper functions to interface with it.
 */
namespace rtt::world_new {
    class FieldComputations {
    public:
        static bool pointIsInDefenceArea(const ai::Field &field, const Vector2 &point, bool isOurDefenceArea = true, double margin = 0.0, bool includeOutsideField = false);
        static bool pointIsInField(const ai::Field &field, const Vector2 &point, double margin = 0.0);  // TODO: Remove margin hack
        static double getPercentageOfGoalVisibleFromPoint(const ai::Field &field, bool ourGoal, const Vector2 &point, const world_new::view::WorldDataView &world, int id = -1, bool ourTeam = false);
        static std::vector<Line> getBlockadesMappedToGoal(const ai::Field &field, bool ourGoal, const Vector2 &point, const world_new::view::WorldDataView &world, int id = -1, bool ourTeam = false);
        static std::vector<Line> mergeBlockades(std::vector<Line> blockades);
        static std::vector<Line> getVisiblePartsOfGoal(const ai::Field &field, bool ourGoal, const Vector2 &point, const world_new::view::WorldDataView &world);
        static Line getGoalSides(const ai::Field &field, bool ourGoal);
        static double getDistanceToGoal(const ai::Field &field, bool ourGoal, const Vector2 &point);
        static Vector2 getPenaltyPoint(const ai::Field &field, bool ourGoal);
        static std::shared_ptr<Vector2> lineIntersectionWithDefenceArea(const ai::Field &field, bool ourGoal, const Vector2 &lineStart, const Vector2 &lineEnd, double margin);
        static double getTotalGoalAngle(const ai::Field &field, bool ourGoal, const Vector2 &point);
        static Polygon getDefenseArea(const ai::Field &field, bool ourDefenseArea = true, double margin = 0.0, bool includeOutSideField = true);
        static Polygon getGoalArea(const ai::Field &field, bool ourGoal = true, double margin = 0.0, bool hasBackMargin = false);
        static Polygon getFieldEdge(const ai::Field &field, double margin = 0.0);
    };

}  // namespace rtt::world_new::fieldComputations

#endif //RTT_FIELDCOMPUTATIONS_HPP
