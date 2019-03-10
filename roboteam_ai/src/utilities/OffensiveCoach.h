//
// Created by robzelluf on 3/8/19.
//

#ifndef ROBOTEAM_AI_OFFENSIVECOACH_H
#define ROBOTEAM_AI_OFFENSIVECOACH_H

#include <roboteam_utils/Vector2.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/utilities/Field.h>

namespace rtt {
namespace ai {
namespace coach {


class OffensiveCoach {
public:
    struct offensivePosition {
        Vector2 position;
        double score;
    };
    static const vector<offensivePosition> &getOffensivePositions();

private:
    static std::vector<offensivePosition> offensivePositions;
    static int maxPositions;
    static std::map<int, int> robotPositions;

    static bool compareByScore(const offensivePosition position1, const offensivePosition position2);
    static double calculateCloseToGoalScore(Vector2 position);
    static double calculateShotAtGoalScore(Vector2 position, roboteam_msgs::World world);
    static double calculatePassLineScore(Vector2 position, roboteam_msgs::World world);
    static double calculateDistanceToOpponentsScore(Vector2 position, roboteam_msgs::World world);
    static double calculateDistanceToTeamScore(Vector2 position, roboteam_msgs::World world);
    static double calculateDistanceFromCorner(Vector2 position, roboteam_msgs::GeometryFieldSize field);
public:
    static double calculatePositionScore(Vector2 position);
    static void calculateNewPositions();
    static void visualizePositions();

    static void setRobot(int robotID);
    static void releaseRobot(int robotID);
    static Vector2 getPositionForRobotID(int robotID);

};

}
}
}


#endif //ROBOTEAM_AI_OFFENSIVECOACH_H
