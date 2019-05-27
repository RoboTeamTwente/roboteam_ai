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

namespace rtt {
namespace ai {
namespace control {

class NumTreePosControl;
class DribbleBackwards;
class DribbleForwards;
class RotateAroundBall;
class RotateAroundRobot;
class BallHandlePosControl {
    private:
        using BallPtr = std::shared_ptr<world::Ball>;
        using RobotPtr = std::shared_ptr<world::Robot>;

        DribbleForwards* dribbleForwards;
        DribbleBackwards* dribbleBackwards;
        RotateAroundRobot* rotateAroundRobot;
        RotateAroundBall* rotateAroundBall;
        NumTreePosControl* numTreePosControl;

        double maxForwardsVelocity = Constants::GRSIM() ? 0.6 : 1.0;
        double maxBackwardsVelocity = Constants::GRSIM() ? 0.3 : 0.8;
        const double errorMargin = 0.02;
        const double angleErrorMargin = 0.02;
        const double maxBallDistance = Constants::ROBOT_RADIUS()*2.0;
        const double targetBallDistance = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
        const double minVelForMovingball = 0.58;
        double ballPlacementAccuracy = 0.04;
        bool canMoveInDefenseArea = false;

        RobotPtr robot;
        BallPtr ball;
        Vector2 targetPos;
        Vector2 finalTargetPos;
        Angle targetAngle;
        Angle finalTargetAngle;

        enum TravelStrategy : short {
          forwards,
          backwards,
          no_preference
        };

        // general functions
        RobotCommand goToBall(bool ballIsFarFromTarget, TravelStrategy preferredTravelStrategy = no_preference);

        int waitingTicks = 0;
        Angle lockedAngle;

        Vector2 previousVelocity = Vector2();

    public:
        explicit BallHandlePosControl(bool canMoveInDefenseArea = false);
        ~BallHandlePosControl();
        void setMaxVelocity(double maxV);
        void setMaxForwardsVelocity(double maxV);
        void setMaxBackwardsVelocity(double maxV);
        RobotCommand getRobotCommand(const RobotPtr &r, const Vector2 &targetP, const Angle &targetA,
                TravelStrategy preferredTravelStrategy = no_preference);
};

} //control
} //ai
} //rtt

#endif //ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H
