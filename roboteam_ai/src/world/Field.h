/*
 * Field
 * This class maintains the FieldGeometry object, which is a message that contains the field geometry
 *  This class also provides helper functions to interface with it.
 */


#ifndef ROBOTEAM_AI_FIELD_H
#define ROBOTEAM_AI_FIELD_H

#include <roboteam_utils/Vector2.h>
#include "roboteam_msgs/GeometryFieldSize.h"
#include <mutex>
#include <thread>
#include <tuple>
#include "WorldData.h"
#include "Robot.h"
#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_utils/Polygon.h>

namespace rtt {
namespace ai {
namespace world {
using Line=std::pair<Vector2, Vector2>;
class Field {
    private:
        roboteam_msgs::GeometryFieldSize field;
        std::mutex fieldMutex;

    public:
        const roboteam_msgs::GeometryFieldSize get_field();
        void set_field(roboteam_msgs::GeometryFieldSize field);
        Vector2 get_our_goal_center();
        Vector2 get_their_goal_center();
        bool pointIsInDefenceArea(const Vector2& point, bool isOurDefenceArea = true, float margin = 0.0,
                bool includeOutsideField = false);
        bool pointIsInField(const Vector2& point, float margin = 0.05); //TODO: Remove margin hack
        double getPercentageOfGoalVisibleFromPoint(bool ourGoal, const Vector2& point,const WorldData &world=world::world->getWorld(), int id = -1, bool ourTeam = false);
        std::vector<Line> getBlockadesMappedToGoal(bool ourGoal, const Vector2& point, const WorldData &world=world::world->getWorld(), int id = -1, bool ourTeam = false);
        std::vector<Line> mergeBlockades(std::vector<Line> blockades);
        std::vector<Line> getVisiblePartsOfGoal(bool ourGoal, const Vector2& point,const WorldData &world=world::world->getWorld());
        Line getGoalSides(bool ourGoal);
        double getDistanceToGoal(bool ourGoal, const Vector2& point);
        Vector2 getPenaltyPoint(bool ourGoal);
        bool lineIntersectsWithDefenceArea(bool ourGoal, const Vector2& lineStart, const Vector2& lineEnd,double margin);
        shared_ptr<Vector2> lineIntersectionWithDefenceArea(bool ourGoal, const Vector2& lineStart, const Vector2& lineEnd,double margin);
        double getTotalGoalAngle(bool ourGoal, const Vector2& point);
        Polygon getDefenseArea(bool ourDefenseArea = true, double margin = 0.0, bool includeOutSideField = true);
        Polygon getGoalArea(bool ourGoal = true, double margin = 0.0, bool hasBackMargin = false);

};

extern Field fieldObj;
extern Field* field;

} // world
} // ai
} // rtt

#endif //ROBOTEAM_AI_FIELD_H
