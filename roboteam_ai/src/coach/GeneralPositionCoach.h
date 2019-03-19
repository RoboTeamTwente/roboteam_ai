//
// Created by mrlukasbos on 19-3-19.
//

#ifndef ROBOTEAM_AI_GENERALPOSITIONCOACH_H
#define ROBOTEAM_AI_GENERALPOSITIONCOACH_H

class GeneralPositionCoach {
    
public:
    explicit GeneralPositionCoach();
    static Vector2 getPositionBehindBallToGoal(double distanceBehindBall, bool ourGoal);
    static Vector2 getPositionBehindBallToRobot(double distanceBehindBall, bool ourRobot, const unsigned int &robotID);
    static Vector2 getPositionBehindBallToPosition(double distanceBehindBall, const Vector2 &position);

    static bool isRobotBehindBallToGoal(double distanceBehindBall, bool ourGoal, const Vector2 &robotPos);
    static bool isRobotBehindBallToRobot(double distanceBehindBall, bool ourRobot, const unsigned int &robotID, const Vector2 &robotPosition);
    static bool isRobotBehindBallToPosition(double distanceBehindBall, const Vector2 &position, const Vector2 &robotPosition);
};

#endif //ROBOTEAM_AI_GENERALPOSITIONCOACH_H
