//
// Created by thijs on 18-12-18.
//

#ifndef ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H
#define ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H

#include <roboteam_utils/Vector2.h>
#include "roboteam_ai/src/control/positionControllers/NumTreePosControl.h"
#include "roboteam_ai/src/control/positionControllers/BasicPosControl.h"
#include "roboteam_ai/src/control/positionControllers/PosController.h"
#include "roboteam_ai/src/control/positionControllers/RobotCommand.h"

#include "DribbleBackwards.h"
#include "DribbleForwards.h"
#include "RotateAroundBall.h"
#include "RotateAroundRobot.h"

namespace rtt {
namespace ai {
namespace control {

class NumTreePosControl;
class DribbleBackwards;
class DribbleForwards;
class BallHandlePosControl {
    private:
        using BallPtr = std::shared_ptr<world::Ball>;
        using RobotPtr = std::shared_ptr<world::Robot>;

        DribbleForwards* dribbleForwards;
        DribbleBackwards* dribbleBackwards;

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

        // general functions
        RobotCommand goToBall(bool ballIsFarFromTarget, TravelStrategy preferredTravelStrategy = no_preference);
        RobotCommand rotateWithBall(RotateStrategy rotateStrategy);
        RobotCommand travelWithBall(TravelStrategy travelStrategy);
        void updateVariables(const RobotPtr &robot, const Vector2 &targetP, const Angle &targetA);

        int waitingTicks = 0;
        Angle lockedAngle;

        Vector2 previousVelocity = Vector2();

    public:
        explicit BallHandlePosControl(bool canMoveInDefenseArea = false);
        ~BallHandlePosControl();
        void setMaxVelocity(double maxV);
        RobotCommand getRobotCommand(const RobotPtr &r, const Vector2 &targetP, const Angle &targetA,
                TravelStrategy preferredTravelStrategy = no_preference);
};

} //control
} //ai
} //rtt

#endif //ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H
