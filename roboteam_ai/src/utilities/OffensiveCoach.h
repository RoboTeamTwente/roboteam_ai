//
// Created by robzelluf on 3/8/19.
//

#ifndef ROBOTEAM_AI_OFFENSIVECOACH_H
#define ROBOTEAM_AI_OFFENSIVECOACH_H

#include <roboteam_utils/Vector2.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/utilities/Field.h>
#include <algorithm>

namespace rtt {
namespace ai {
namespace coach {


class OffensiveCoach {
public:
    struct OffensivePosition {
        Vector2 position;
        double score;
    };

    static void calculateNewPositions();
    static void calculateNewRobotPositions(std::shared_ptr<roboteam_msgs::WorldRobot> robot);

    static Vector2 calculatePositionForRobot(std::shared_ptr<roboteam_msgs::WorldRobot> robot);
    static void releaseRobot(int robotID);
    static Vector2 getPositionForRobotID(int robotID);
    static std::vector<OffensivePosition> getRobotPositionVectors();
    static int getBestStrikerID();


private:

    static double marginFromLines;
    static double maxDistanceFromBall;

    static std::vector<OffensivePosition> offensivePositions;
public:
    static const vector<OffensivePosition> &getOffensivePositions();

private:
    static int maxPositions;
    static std::map<int, OffensivePosition> robotPositions;
public:
    static const map<int, OffensivePosition> &getRobotPositions();

private:

    static bool compareByScore(OffensivePosition position1, OffensivePosition position2);
    static double calculateCloseToGoalScore(Vector2 position);
    static double calculateShotAtGoalScore(Vector2 position, roboteam_msgs::World world);
    static double calculatePassLineScore(Vector2 position, roboteam_msgs::World world);
    static double calculateDistanceToOpponentsScore(Vector2 position, roboteam_msgs::World world);
    static double calculateDistanceFromCorner(Vector2 position, roboteam_msgs::GeometryFieldSize field);
    static double calculateDistanceFromBallScore(Vector2 position, roboteam_msgs::GeometryFieldSize& field, roboteam_msgs::WorldBall& ball);
    static double calculatePositionScore(Vector2 position);
    static void drawOffensivePoints();
    static void recalculateOffensivePositions();
    static OffensivePosition calculateRandomPosition(double xStart, double xEnd, double yStart, double yEnd);
    static bool positionTooCloseToRobotPositions(OffensivePosition position, int self = -1);

    static void compareToCurrentPositions(const OffensivePosition &position);

    static Vector2 getClosestOffensivePosition(const shared_ptr<roboteam_msgs::WorldRobot> &robot);
};

}
}
}


#endif //ROBOTEAM_AI_OFFENSIVECOACH_H
