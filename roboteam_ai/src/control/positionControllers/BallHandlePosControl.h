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

        double maxForwardsVelocity = Constants::GRSIM() ? 0.6 : 1.0;
        const double errorMargin = 0.02;
        const double angleErrorMargin = 0.02;
        const double maxBallDistance = Constants::ROBOT_RADIUS()*2.0;
        const double targetBallDistance = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
        const double robotRadius = Constants::ROBOT_RADIUS();
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
        double ballPlacementAccuracy = 0.01;

        NumTreePosControl numTreePosController = NumTreePosControl();
        enum RotateStrategy : short {
          rotateAroundBall,
          rotateAroundRobot
        };
        void printRotateStrategy(RotateStrategy strategy);

        enum TravelStrategy : short {
          forwards,
          backwards
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
        int B_count = 0;
        Vector2 B_approachPosition;
        Angle B_lockedAngle;
        std::pair<Vector2, Vector2> B_backwardsDribbleLine;

        // functions for backwards progress
        void updateBackwardsProgress();
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
        Angle F_lockedAngle;
        std::pair<Vector2, Vector2> F_forwardsDribbleLine;

        // functions for forwards progress
        void updateForwardsProgress();
        RobotCommand F_startTravelForwards();
        RobotCommand F_sendTurnCommand();
        RobotCommand F_sendApproachCommand();
        RobotCommand F_sendDribbleForwardsCommand();

        // general functions
        RobotCommand goToBall(bool ballIsFarFromTarget);
        RobotCommand rotateWithBall(RotateStrategy rotateStrategy);
        RobotCommand travelWithBall(TravelStrategy travelStrategy);
        void updateVariables(const RobotPtr &robot, const Vector2 &targetP, const Angle &targetA);

        // limit velocity & acceleration
        Vector2 previousVelocity = Vector2();
        RobotCommand limitCommand(RobotCommand command);
    public:
        explicit BallHandlePosControl(bool canMoveInDefenseArea = false);

        void setMaxVelocity(double maxV);
        RobotCommand getPosVelAngle(const RobotPtr &r, const Vector2 &targetP, const Angle &targetA);
        RobotCommand F_sendSuccessCommand();
};

} //control
} //ai
} //rtt

#endif //ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H
