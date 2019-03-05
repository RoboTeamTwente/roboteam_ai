/*
 * Field.h
 * This class maintains the FieldGeometry object, which is a message that contains the field geometry
 *  This class also provides helper functions to interface with it.
 */


#ifndef ROBOTEAM_AI_FIELD_H
#define ROBOTEAM_AI_FIELD_H

#include <roboteam_utils/Vector2.h>
#include "roboteam_msgs/GeometryFieldSize.h"
#include <mutex>
#include <thread>
#include "World.h"

namespace rtt {
namespace ai {

class Field {
    private:
        static roboteam_msgs::GeometryFieldSize field;
        static std::mutex fieldMutex;

    public:
        static const roboteam_msgs::GeometryFieldSize get_field();
        static void set_field(roboteam_msgs::GeometryFieldSize field);
        static Vector2 get_our_goal_center();
        static Vector2 get_their_goal_center();
        static bool pointIsInDefenceArea(Vector2 point, bool isOurDefenceArea = true, float margin = 0.0);
        static bool pointIsInField(Vector2 point, float margin = 0.0);
        static int getRobotClosestToGoal(bool ourRobot, bool ourGoal);
        static double getPercentageOfGoalVisibleFromPoint(bool ourGoal, Vector2 point, std::vector<roboteam_msgs::WorldRobot> botsToCheck=World::getAllRobots(), double collisionRadius=Constants::ROBOT_RADIUS());
        static std::vector<std::pair<Vector2, Vector2>> getBlockadesMappedToGoal(bool ourGoal, Vector2 point, std::vector<roboteam_msgs::WorldRobot> botsToCheck=World::getAllRobots(),double collisionRadius=Constants::ROBOT_RADIUS());
        static std::vector<std::pair<Vector2, Vector2>> mergeBlockades(std::vector<std::pair<Vector2, Vector2>> blockades);
        static std::vector<std::pair<Vector2, Vector2>> getVisiblePartsOfGoal(bool ourGoal, Vector2 point,  std::vector<roboteam_msgs::WorldRobot> botsToCheck=World::getAllRobots(), double collisionRadius=Constants::ROBOT_RADIUS());
        static std::pair<Vector2, Vector2> getGoalSides(bool ourGoal);
        static double getTotalGoalAngle(bool ourGoal, Vector2 point);
        static double getTotalVisibleGoalAngle(bool ourGoal, Vector2 point, std::vector<roboteam_msgs::WorldRobot> botsToCheck=World::getAllRobots(),double collisionRadius=Constants::ROBOT_RADIUS());
        static std::shared_ptr<Vector2> lineIntersectsWithDefenceArea(bool ourGoal,Vector2 lineStart,Vector2 lineEnd,double margin);
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_FIELD_H
