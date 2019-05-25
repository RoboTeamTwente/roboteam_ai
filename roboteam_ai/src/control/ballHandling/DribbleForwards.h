//
// Created by thijs on 25-5-19.
//

#ifndef ROBOTEAM_AI_DRIBBLEFORWARDS_H
#define ROBOTEAM_AI_DRIBBLEFORWARDS_H

#include <roboteam_utils/Angle.h>
#include <roboteam_utils/Vector2.h>
#include "roboteam_ai/src/control/positionControllers/RobotCommand.h"
#include "roboteam_ai/src/world/Robot.h"

namespace rtt {
namespace ai {
namespace control {

class DribbleBackwards;
class RotateAroundBall;
class RotateAroundRobot;
class DribbleForwards {
    public:
        enum ForwardsProgress : short {
          F_start,
          F_turning,
          F_approaching,
          F_dribbleForward,
          F_success,
          F_fail
        };
        ForwardsProgress getForwardsProgression();

    private:
        RotateAroundBall* rotateAroundBall;
        RotateAroundRobot* rotateAroundRobot;

        using RobotPtr = world::Robot::RobotPtr;
        using BallPtr = world::Ball::BallPtr;
        RobotPtr robot;
        BallPtr ball;

        ForwardsProgress forwardsProgress = F_start;
        void printForwardsProgress();

        // variables for forwards progress
        std::pair<Vector2, Vector2> F_forwardsDribbleLine;
        Angle lockedAngle;
        Angle targetAngle;
        Angle finalTargetAngle;
        Vector2 targetPos;
        Vector2 finalTargetPos;

        int waitingTicks;
        const double errorMargin;
        const double angleErrorMargin;
        const double ballPlacementAccuracy;
        const double maxVel;

        // functions for forwards progress
        void updateForwardsProgress();
        RobotCommand sendForwardsCommand();
        RobotCommand F_startTravelForwards();
        RobotCommand F_sendTurnCommand();
        RobotCommand F_sendApproachCommand();
        RobotCommand F_sendDribbleForwardsCommand();
        RobotCommand F_sendSuccessCommand();

    public:
        RobotCommand getRobotCommand(const world::Robot::RobotPtr &r,
                const Vector2 &targetP, const Angle &targetA);
        void reset();

        explicit DribbleForwards(double errorMargin = 0.02, double angularErrorMargin = 0.02,
                double ballPlacementAccuracy = 0.04, double maxVel = 0.7);

};

}
}
}

#endif //ROBOTEAM_AI_DRIBBLEFORWARDS_H
