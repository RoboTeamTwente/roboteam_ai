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

class RotateAroundBall;
class RotateAroundRobot;
class DribbleBackwards {
    public:
        enum BackwardsProgress : short {
          START,
          TURNING,
          APPROACHING,
          OVERSHOOTING,
          DRIBBLING,
          DRIBBLE_BACKWARDS,
          SUCCESS,
          FAIL
        };
        BackwardsProgress getBackwardsProgression();

    private:
        RotateAroundBall* rotateAroundBall;
        RotateAroundRobot* rotateAroundRobot;

        using RobotPtr = world::Robot::RobotPtr;
        using BallPtr = world::Ball::BallPtr;
        RobotPtr robot;
        BallPtr ball;

        BackwardsProgress backwardsProgress = START;
        void printBackwardsProgress();

        // variables for backwards progress
        Vector2 approachPosition;
        std::pair<Vector2, Vector2> backwardsDribbleLine;
        Angle lockedAngle;
        Angle targetAngle;
        Angle finalTargetAngle;
        Vector2 targetPos;
        Vector2 finalTargetPos;

        // error margins and accuracy
        int waitingTicks;
        double errorMargin;
        double angleErrorMargin;
        double ballPlacementAccuracy;
        double maxVel;
    public:
        void setMaxVel(double maxVel);
    private:

        // functions for backwards progress
        void updateBackwardsProgress();
        RobotCommand sendBackwardsCommand();
        RobotCommand startTravelBackwards();
        RobotCommand sendTurnCommand();
        RobotCommand sendApproachCommand();
        RobotCommand sendOvershootCommand();
        RobotCommand sendDribblingCommand();
        RobotCommand sendDribbleBackwardsCommand();
        RobotCommand sendSuccessCommand();

    public:
        RobotCommand getRobotCommand(const world::Robot::RobotPtr &r,
                const Vector2 &targetP, const Angle &targetA);
        void reset();

        explicit DribbleBackwards(double errorMargin = 0.02, double angularErrorMargin = 0.02,
                double ballPlacementAccuracy = 0.04, double maxVel = 0.4);
        ~DribbleBackwards();
};

}
}
}

#endif //ROBOTEAM_AI_DRIBBLEBACKWARDS_H
