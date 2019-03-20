//
// Created by mrlukasbos on 19-3-19.
//


#ifndef ROBOTEAM_AI_GENERALPOSITIONCOACH_H
#define ROBOTEAM_AI_GENERALPOSITIONCOACH_H

#include "roboteam_utils/Vector2.h"

namespace rtt {
namespace ai {
namespace coach {

class GeneralPositionCoach {

public:
    explicit GeneralPositionCoach() = default;
    Vector2 getPositionBehindBallToGoal(double distanceBehindBall, bool ourGoal);
    Vector2 getPositionBehindBallToRobot(double distanceBehindBall, bool ourRobot, const unsigned int &robotID);
    Vector2 getPositionBehindBallToPosition(double distanceBehindBall, const Vector2 &position);

    bool isRobotBehindBallToGoal(double distanceBehindBall, bool ourGoal, const Vector2 &robotPos);
    bool isRobotBehindBallToRobot(double distanceBehindBall, bool ourRobot, const unsigned int &robotID, const Vector2 &robotPosition);
    bool isRobotBehindBallToPosition(double distanceBehindBall, const Vector2 &position, const Vector2 &robotPosition);
    Vector2 getDemoKeeperGetBallPos(Vector2 ballPos);

};

extern GeneralPositionCoach g_generalPositionCoach;

} // coach
} // ai
} // rtt

#endif //ROBOTEAM_AI_GENERALPOSITIONCOACH_H
