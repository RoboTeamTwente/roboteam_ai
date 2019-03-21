//
// Created by robzelluf on 3/21/19.
//

#ifndef ROBOTEAM_AI_COACHHEURISTICS_H
#define ROBOTEAM_AI_COACHHEURISTICS_H

#include <roboteam_utils/Vector2.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/utilities/Field.h>

namespace rtt {
namespace ai {
namespace coach {

class CoachHeuristics {
private:
    static double maxDistanceFromBall;
public:
    static double calculateCloseToGoalScore(Vector2 position);
    static double calculateShotAtGoalScore(Vector2 position, roboteam_msgs::World world);
    static double calculatePassLineScore(Vector2 position, roboteam_msgs::World world);
    static double calculateDistanceToOpponentsScore(Vector2 position, roboteam_msgs::World world);
    static double calculateDistanceFromCornerScore(Vector2 position, roboteam_msgs::GeometryFieldSize field);
    static double calculateDistanceFromBallScore(Vector2 position, roboteam_msgs::GeometryFieldSize& field, roboteam_msgs::WorldBall& ball);
    static double calculatePositionScore(Vector2 position);
};

}
}
}


#endif //ROBOTEAM_AI_COACHHEURISTICS_H
