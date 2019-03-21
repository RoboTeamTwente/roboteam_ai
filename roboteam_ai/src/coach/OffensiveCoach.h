//
// Created by robzelluf on 3/21/19.
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

    void calculateNewPositions();
    void calculateNewRobotPositions(std::shared_ptr<roboteam_msgs::WorldRobot> robot);

    Vector2 calculatePositionForRobot(std::shared_ptr<roboteam_msgs::WorldRobot> robot);
    void releaseRobot(int robotID);
    Vector2 getPositionForRobotID(int robotID);
    std::vector<OffensivePosition> getRobotPositionVectors();
    int getBestStrikerID();
    const vector<OffensivePosition> &getOffensivePositions();
    const map<int, OffensivePosition> &getRobotPositions();


private:

    double marginFromLines = 0.2;
    double maxDistanceFromBall = 6.0;

    std::vector<OffensivePosition> offensivePositions;
    int maxPositions = 4;
    std::map<int, OffensivePosition> robotPositions;

    static bool compareByScore(OffensivePosition position1, OffensivePosition position2);
    double calculateCloseToGoalScore(Vector2 position);
    double calculateShotAtGoalScore(Vector2 position, roboteam_msgs::World world);
    double calculatePassLineScore(Vector2 position, roboteam_msgs::World world);
    double calculateDistanceToOpponentsScore(Vector2 position, roboteam_msgs::World world);
    double calculateDistanceFromCorner(Vector2 position, roboteam_msgs::GeometryFieldSize field);
    double calculateDistanceFromBallScore(Vector2 position, roboteam_msgs::GeometryFieldSize& field, roboteam_msgs::WorldBall& ball);
    double calculatePositionScore(Vector2 position);
    void drawOffensivePoints();
    void recalculateOffensivePositions();
    OffensivePosition calculateRandomPosition(double xStart, double xEnd, double yStart, double yEnd);
    bool positionTooCloseToRobotPositions(OffensivePosition position, int self = -1);

    void compareToCurrentPositions(const OffensivePosition &position);

    Vector2 getClosestOffensivePosition(const shared_ptr<roboteam_msgs::WorldRobot> &robot);
};

extern OffensiveCoach g_offensiveCoach;

}
}
}


#endif //ROBOTEAM_AI_OFFENSIVECOACH_H
