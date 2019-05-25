//
// Created by thijs on 25-5-19.
//

#ifndef ROBOTEAM_AI_DRIBBLEBACKWARDS_H
#define ROBOTEAM_AI_DRIBBLEBACKWARDS_H

#include <roboteam_utils/Angle.h>
#include <roboteam_utils/Vector2.h>
#include "roboteam_ai/src/control/positionControllers/RobotCommand.h"
#include "roboteam_ai/src/world/Robot.h"

namespace rtt {
namespace ai {
namespace control {

class DribbleForwards;
class RotateAroundBall;
class RotateAroundRobot;
class DribbleBackwards {
    public:
        enum BackwardsProgress : short {
          B_start,
          B_turning,
          B_approaching,
          B_overshooting,
          B_dribbling,
          B_dribbleBackwards,
          B_success,
          B_fail
        };
        BackwardsProgress getBackwardsProgression();

    private:
        RotateAroundBall* rotateAroundBall;
        RotateAroundRobot* rotateAroundRobot;

        using RobotPtr = world::Robot::RobotPtr;
        using BallPtr = world::Ball::BallPtr;
        RobotPtr robot;
        BallPtr ball;

        BackwardsProgress backwardsProgress = B_start;
        void printBackwardsProgress();

        // variables for backwards progress
        Vector2 B_approachPosition;
        std::pair<Vector2, Vector2> B_backwardsDribbleLine;
        Angle lockedAngle;
        Angle targetAngle;
        Angle finalTargetAngle;
        Vector2 targetPos;
        Vector2 finalTargetPos;

        // error margins and accuracy
        int waitingTicks;
        const double errorMargin;
        const double angleErrorMargin;
        const double ballPlacementAccuracy;
        const double maxVel;

        // functions for backwards progress
        void updateBackwardsProgress();
        RobotCommand sendBackwardsCommand();
        RobotCommand B_startTravelBackwards();
        RobotCommand B_sendTurnCommand();
        RobotCommand B_sendApproachCommand();
        RobotCommand B_sendOvershootCommand();
        RobotCommand B_sendDribblingCommand();
        RobotCommand B_sendDribbleBackwardsCommand();
        RobotCommand B_sendSuccessCommand();

    public:
        RobotCommand getRobotCommand(const world::Robot::RobotPtr &r,
                const Vector2 &targetP, const Angle &targetA);
        void reset();

        explicit DribbleBackwards(double errorMargin = 0.02, double angularErrorMargin = 0.02,
                double ballPlacementAccuracy = 0.04, double maxVel = 0.4);

};

}
}
}

#endif //ROBOTEAM_AI_DRIBBLEBACKWARDS_H
