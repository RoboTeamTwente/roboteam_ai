/*
 * FieldComputations
 * This class maintains the FieldGeometry object, which is a message that contains the field geometry
 *  This class also provides helper functions to interface with it.
 */

#ifndef ROBOTEAM_AI_FIELD_H
#define ROBOTEAM_AI_FIELD_H


#include <roboteam_utils/Polygon.h>
#include "mutex"
#include <cmath>
#include "roboteam_proto/messages_robocup_ssl_geometry.pb.h"
#include "roboteam_proto/GeometryFieldSize.pb.h"
#include <include/roboteam_ai/world/FieldMessage.h>

namespace rtt {
namespace ai {
namespace world {
    class WorldData;
} // world


class FieldComputations {
    public:
        static bool pointIsInDefenceArea(FieldMessage &field, const Vector2& point, bool isOurDefenceArea = true,
                double margin = 0.0, bool includeOutsideField = false);
        static bool pointIsInField(FieldMessage &field, const Vector2& point, double margin = 0.0); //TODO: Remove margin hack
        static double getPercentageOfGoalVisibleFromPoint(FieldMessage &field, bool ourGoal, const Vector2& point,
                const world::WorldData &world, int id = -1, bool ourTeam = false);
        static std::vector<Line> getBlockadesMappedToGoal(FieldMessage &field, bool ourGoal, const Vector2& point,
                const world::WorldData &world, int id = -1, bool ourTeam = false);
        static std::vector<Line> mergeBlockades(std::vector<Line> blockades);
        static std::vector<Line> getVisiblePartsOfGoal(FieldMessage &field, bool ourGoal, const Vector2& point,
                const world::WorldData &world);
        static Line getGoalSides(FieldMessage &field, bool ourGoal);
        static double getDistanceToGoal(FieldMessage &field, bool ourGoal, const Vector2& point);
        static Vector2 getPenaltyPoint(FieldMessage &field, bool ourGoal);
        static std::shared_ptr<Vector2> lineIntersectionWithDefenceArea(FieldMessage &field, bool ourGoal,
                const Vector2& lineStart, const Vector2& lineEnd,double margin);
        static double getTotalGoalAngle(FieldMessage &field, bool ourGoal, const Vector2& point);
        static Polygon getDefenseArea(FieldMessage &field, bool ourDefenseArea = true, double margin = 0.0,
                bool includeOutSideField = true);
        static Polygon getGoalArea(FieldMessage &field, bool ourGoal = true, double margin = 0.0, bool hasBackMargin = false);
        static Polygon getFieldEdge(FieldMessage &field, double margin = 0.0);
};

extern FieldComputations fieldObj;
extern FieldComputations* field;

} // ai
} // rtt

#endif //ROBOTEAM_AI_FIELD_H
