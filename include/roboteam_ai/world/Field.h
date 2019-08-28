/*
 * Field
 * This class maintains the FieldGeometry object, which is a message that contains the field geometry
 *  This class also provides helper functions to interface with it.
 */

#ifndef ROBOTEAM_AI_FIELD_H
#define ROBOTEAM_AI_FIELD_H


#include <roboteam_utils/Polygon.h>
#include <GeometryFieldSize.pb.h>
#include "mutex"
#include <cmath>

namespace rtt {
namespace ai {
namespace world {

using Line=std::pair<Vector2, Vector2>;

class WorldData;
class Field {
    private:
        roboteam_proto::GeometryFieldSize field;
        std::mutex fieldMutex;

    public:
        const roboteam_proto::GeometryFieldSize get_field();
        void set_field(roboteam_proto::GeometryFieldSize field);
        Vector2 get_our_goal_center();
        Vector2 get_their_goal_center();
        bool pointIsInDefenceArea(const Vector2& point, bool isOurDefenceArea = true, double margin = 0.0,
                bool includeOutsideField = false);
        bool pointIsInField(const Vector2& point, double margin = 0.0); //TODO: Remove margin hack
        double getPercentageOfGoalVisibleFromPoint(bool ourGoal, const Vector2& point,const WorldData &world, int id = -1, bool ourTeam = false);
        std::vector<Line> getBlockadesMappedToGoal(bool ourGoal, const Vector2& point, const WorldData &world, int id = -1, bool ourTeam = false);
        std::vector<Line> mergeBlockades(std::vector<Line> blockades);
        std::vector<Line> getVisiblePartsOfGoal(bool ourGoal, const Vector2& point,const WorldData &world);
        Line getGoalSides(bool ourGoal);
        double getDistanceToGoal(bool ourGoal, const Vector2& point);
        Vector2 getPenaltyPoint(bool ourGoal);
        bool lineIntersectsWithDefenceArea(bool ourGoal, const Vector2& lineStart, const Vector2& lineEnd,double margin);
        std::shared_ptr<Vector2> lineIntersectionWithDefenceArea(bool ourGoal, const Vector2& lineStart, const Vector2& lineEnd,double margin);
        double getTotalGoalAngle(bool ourGoal, const Vector2& point);
        Polygon getDefenseArea(bool ourDefenseArea = true, double margin = 0.0, bool includeOutSideField = true);
        Polygon getGoalArea(bool ourGoal = true, double margin = 0.0, bool hasBackMargin = false);
        Polygon getFieldEdge(double margin = 0.0);
};

extern Field fieldObj;
extern Field* field;

} // world
} // ai
} // rtt

#endif //ROBOTEAM_AI_FIELD_H
