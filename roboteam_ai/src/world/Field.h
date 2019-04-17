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
        bool pointIsInDefenseArea(const Vector2 &point, bool isOurDefenseArea = true, float margin = 0.0,
                bool outsideField = false);
        bool pointIsInField(const Vector2& point, float margin = 0.05); //TODO: Remove margin hack
        int getRobotClosestToGoal(WhichRobots whichRobots, bool ourGoal);
        double getPercentageOfGoalVisibleFromPoint(bool ourGoal, const Vector2& point,const WorldData &world=world::world->getWorld());
        std::vector<Line> getBlockadesMappedToGoal(bool ourGoal, const Vector2& point, const WorldData &world=world::world->getWorld());
        std::vector<Line> mergeBlockades(std::vector<Line> blockades);
        std::vector<Line> getVisiblePartsOfGoal(bool ourGoal, const Vector2& point,const WorldData &world=world::world->getWorld());
        Line getGoalSides(bool ourGoal);
        double getDistanceToGoal(bool ourGoal, const Vector2& point);
        Vector2 getPenaltyPoint(bool ourGoal);
        std::shared_ptr<Vector2> lineIntersectsWithDefenceArea(bool ourGoal, const Vector2& lineStart, const Vector2& lineEnd,double margin);
        double getTotalGoalAngle(bool ourGoal, const Vector2& point);
        std::vector<Vector2> getDefenseArea(bool ourDefenseArea = true, double margin = 0.0);

};

extern Field fieldObj;
extern Field* field;

} // world
} // ai
} // rtt

#endif //ROBOTEAM_AI_FIELD_H
