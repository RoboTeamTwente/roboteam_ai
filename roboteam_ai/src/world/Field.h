/*
 * Field.h
 * This class maintains the FieldGeometry object, which is a message that contains the field geometry
 *  This class also provides helper functions to interface with it.
 */


#ifndef ROBOTEAM_AI_FIELDD_H
#define ROBOTEAM_AI_FIELDD_H

#include <roboteam_utils/Vector2.h>
#include "roboteam_msgs/GeometryFieldSize.h"
#include <mutex>
#include <thread>
#include "WorldData.h"
#include <roboteam_ai/src/control/ControlUtils.h>

namespace rtt {
namespace ai {
namespace world {

class Field {
    private:
        roboteam_msgs::GeometryFieldSize field;
        std::mutex fieldMutex;

    public:
        const roboteam_msgs::GeometryFieldSize get_field();
        void set_field(roboteam_msgs::GeometryFieldSize field);
        Vector2 get_our_goal_center();
        Vector2 get_their_goal_center();
        bool pointIsInDefenceArea(Vector2 point, bool isOurDefenceArea = true, float margin = 0.0,
                bool outsideField = false);
        bool pointIsInField(Vector2 point, float margin = 0.05); //TODO: Remove margin hack
        int getRobotClosestToGoal(WhichRobots whichRobots, bool ourGoal);
        double getPercentageOfGoalVisibleFromPoint(bool ourGoal, Vector2 point);
        std::vector<std::pair<Vector2, Vector2>> getBlockadesMappedToGoal(bool ourGoal, Vector2 point);
        std::vector<std::pair<Vector2, Vector2>> mergeBlockades(std::vector<std::pair<Vector2, Vector2>> blockades);
        std::vector<std::pair<Vector2, Vector2>> getVisiblePartsOfGoal(bool ourGoal, Vector2 point);
        std::pair<Vector2, Vector2> getGoalSides(bool ourGoal);
};

extern Field fieldObj;
extern Field* field;

} // world
} // ai
} // rtt

#endif //ROBOTEAM_AI_FIELD_H
