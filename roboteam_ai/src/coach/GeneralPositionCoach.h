//
// Created by mrlukasbos on 19-3-19.
//


#ifndef ROBOTEAM_AI_GENERALPOSITIONCOACH_H
#define ROBOTEAM_AI_GENERALPOSITIONCOACH_H

#include "roboteam_utils/Vector2.h"
#include <roboteam_ai/src/utilities/Constants.h>

namespace rtt {
namespace ai {
namespace coach {

class GeneralPositionCoach {
public:
    explicit GeneralPositionCoach() = default;
    Vector2 getPositionBehindBallToGoal(double distanceBehindBall, bool ourGoal);
    Vector2 getPositionBehindBallToRobot(double distanceBehindBall, bool ourRobot, const unsigned int &robotID);
    Vector2 getPositionBehindBallToPosition(double distanceBehindBall, const Vector2 &position);
    Vector2 getPositionBehindPositionToPosition(double distanceBehindBall, const Vector2 &behindPosition, const Vector2 &toPosition);

    bool isRobotBehindBallToGoal(double distanceBehindBall, bool ourGoal, const Vector2 &robotPos, double angleMargin = 0.10);
    bool isRobotBehindBallToRobot(double distanceBehindBall, bool ourRobot, const unsigned int &robotID, const Vector2 &robotPosition, double angleMargin = 0.15);
    bool isRobotBehindBallToPosition(double distanceBehindBall, const Vector2 &position, const Vector2 &robotPosition, double angleMargin = 0.15);
    Vector2 getDemoKeeperGetBallPos(Vector2 ballPos);
    static std::vector<Vector2> getPenaltyPositions(int number);
    static std::vector<Vector2> getFreeKickPositions(int number);
    static std::vector<Vector2> getDefendFreeKick(int number);
    static std::vector<Vector2> getDefendPenaltyPositions(int number);

};

extern GeneralPositionCoach g_generalPositionCoach;

} // coach
} // ai
} // rtt

#endif //ROBOTEAM_AI_GENERALPOSITIONCOACH_H
