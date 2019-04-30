//
// Created by thijs on 18-12-18.
//

#ifndef ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H
#define ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H

#include <roboteam_utils/Vector2.h>
#include "NumTreePosControl.h"
#include "BasicPosControl.h"
#include "PosController.h"
#include "RobotCommand.h"

namespace rtt {
namespace ai {
namespace control {

class NumTreePosControl;
class BallHandlePosControl {
    private:
        using BallPtr = std::shared_ptr<world::Ball>;
        using RobotPtr = std::shared_ptr<world::Robot>;

        const double errorMargin = 0.02;
        const double angleErrorMargin = 0.02;
        const double maxBallDistance = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS()*3.0;
        const double targetBallDistance = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
        const double robotRadius = Constants::ROBOT_RADIUS();
        const double maxForwardsVelocity = Constants::GRSIM() ? 0.6 : 1.0;
        const double maxBackwardsVelocity = Constants::GRSIM() ? 0.3 : 0.8;
        const double maxAngularVelocity = 0.2;
        bool canMoveInDefenseArea = false;

        RobotPtr robot;
        BallPtr ball;
        Vector2 robotToBall;
        Vector2 ballToRobot;
        Vector2 targetPos;
        Vector2 finalTargetPos;
        Angle targetAngle;
        Angle finalTargetAngle;

        NumTreePosControl numTreePosController = NumTreePosControl();
        enum RotateStrategy : short {
          rotateAroundBall,
          rotateAroundRobot,
          fastest,
          safest,
          defaultRotate
        };
        enum TravelStrategy : short {
          forwards,
          backwards,
          defaultTravel
        };
        enum BackwardsProgress : short {
          start,
          turning,
          approaching,
          overshooting,
          dribbling,
          dribbleBackwards,
          success,
          fail
        };
        BackwardsProgress backwardsProgress = start;

        int count = 0;
        Vector2 approachPosition;
        Angle lockedAngle;
        std::pair<Vector2, Vector2> backwardsDribbleLine;
        void updateBackwardsProgress();
        RobotCommand startTravelBackwards();
        RobotCommand sendTurnCommand();
        RobotCommand sendApproachCommand();
        RobotCommand sendOvershootCommand();
        RobotCommand sendDribblingCommand();
        RobotCommand sendDribbleBackwardsCommand();

        RobotCommand rotateWithBall(RotateStrategy rotateStrategy);
        RobotCommand travelWithBall(TravelStrategy travelStrategy);

        Vector2 previousVelocity = Vector2();
        RobotCommand limitCommand(RobotCommand command);

    public:
        explicit BallHandlePosControl(bool canMoveInDefenseArea = false);

        RobotCommand getPosVelAngle(const RobotPtr &robot, const Vector2 &target, const Angle &targetAngle);
};

} //control
} //ai
} //rtt

#endif //ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H
