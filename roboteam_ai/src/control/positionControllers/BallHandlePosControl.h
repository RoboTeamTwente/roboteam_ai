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

        const double robotRadius = Constants::ROBOT_RADIUS();
        const double ballRadius = Constants::BALL_RADIUS();

        double maxForwardsVelocity = Constants::GRSIM() ? 0.6 : 1.0;
        double maxBackwardsVelocity = Constants::GRSIM() ? 0.3 : 0.8;
        const double errorMargin = 0.02;
        const double angleErrorMargin = 0.02;
        const double maxBallDistance = robotRadius*2.0;
        const double targetBallDistance = robotRadius + ballRadius;
        const double maxAngularVelocity = 0.2;
        const double minVelForMovingball = 0.58;
        bool canMoveInDefenseArea = false;

        RobotPtr robot;
        BallPtr ball;
        Vector2 robotToBall;
        Vector2 ballToRobot;
        Vector2 targetPos;
        Vector2 finalTargetPos;
        Angle targetAngle;
        Angle finalTargetAngle;
        double ballPlacementAccuracy = 0.04;

        NumTreePosControl numTreePosController = NumTreePosControl();
        enum RotateStrategy : short {
          rotateAroundBall,
          rotateAroundRobot
        };
        void printRotateStrategy(RotateStrategy strategy);

        enum TravelStrategy : short {
          forwards,
          backwards,
          no_preference
        };
        void printTravelStrategy(TravelStrategy strategy);

        // backwards progress
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
        BackwardsProgress backwardsProgress = B_start;
        void printBackwardsProgress();

        // variables for backwards progress
        Vector2 B_approachPosition;
        std::pair<Vector2, Vector2> B_backwardsDribbleLine;

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

        // forwards progress
        enum ForwardsProgress : short {
          F_start,
          F_turning,
          F_approaching,
          F_dribbleForward,
          F_success,
          F_fail
        };
        ForwardsProgress forwardsProgress = F_start;
        void printForwardsProgress();

        // variables for forwards progress
        std::pair<Vector2, Vector2> F_forwardsDribbleLine;

        // functions for forwards progress
        void updateForwardsProgress();
        RobotCommand sendForwardsCommand();
        RobotCommand F_startTravelForwards();
        RobotCommand F_sendTurnCommand();
        RobotCommand F_sendApproachCommand();
        RobotCommand F_sendDribbleForwardsCommand();

        // general functions
        RobotCommand goToBall(bool ballIsFarFromTarget, TravelStrategy preferredTravelStrategy = no_preference);
        RobotCommand rotateWithBall(RotateStrategy rotateStrategy);
        RobotCommand travelWithBall(TravelStrategy travelStrategy);
        void updateVariables(const RobotPtr &robot, const Vector2 &targetP, const Angle &targetA);

        int waitingTicks = 0;
        Angle lockedAngle;

        // limit velocity & acceleration
        Vector2 previousVelocity = Vector2();
        RobotCommand limitCommand(RobotCommand command);
    public:
        explicit BallHandlePosControl(bool canMoveInDefenseArea = false);

        void setMaxVelocity(double maxV);
        RobotCommand getRobotCommand(const RobotPtr &r, const Vector2 &targetP, const Angle &targetA,
                TravelStrategy preferredTravelStrategy = no_preference);
        RobotCommand F_sendSuccessCommand();
};

} //control
} //ai
} //rtt

#endif //ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H
