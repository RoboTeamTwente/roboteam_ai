//
// Created by thijs on 18-12-18.
//

#ifndef ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H
#define ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H

#include <roboteam_utils/Vector2.h>
#include "roboteam_ai/src/control/numTrees/NumTreePosControl.h"
#include "roboteam_ai/src/control/positionControllers/RobotCommand.h"
#include <roboteam_ai/src/utilities/Constants.h>

namespace rtt {
namespace ai {
namespace control {

class DribbleBackwards;
class DribbleForwards;
class RotateAroundBall;
class RotateWithBall;
class BallHandlePosControl : public NumTreePosControl {
    private:
        using BallPtr = std::shared_ptr<world::Ball>;
        using RobotPtr = std::shared_ptr<world::Robot>;

        DribbleForwards* dribbleForwards;
        DribbleBackwards* dribbleBackwards;
        RotateWithBall* rotateWithBall;
        RotateAroundBall* rotateAroundBall;

        double maxForwardsVelocity = Constants::GRSIM() ? 0.6 : 1.0;
        double maxBackwardsVelocity = Constants::GRSIM() ? 0.3 : 0.8;
        const double errorMargin = 0.02;
        const double angleErrorMargin = 0.02;
        const double maxBallDistance = Constants::ROBOT_RADIUS()*2.0;
        const double targetBallDistance = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
        const double minVelForMovingball = 0.12;
        double ballPlacementAccuracy = 0.04;

        RobotPtr robot;
        BallPtr ball;
        Vector2 targetPos;
        Vector2 finalTargetPos;
        Vector2 targetPosToCatchBall = Vector2();
        Angle targetAngle;
        Angle finalTargetAngle;
        Angle lockedAngle = 0;

    public:
        enum TravelStrategy : short {
          FORWARDS,
          BACKWARDS,
          NO_PREFERENCE
        };
    private:
        TravelStrategy preferredTravelStrategy = NO_PREFERENCE;

        // general functions
        RobotCommand handleBall(const Vector2 &targetBallPos, TravelStrategy travelStrategy,
                bool shouldGoToBall, bool ballIsFarFromTarget = true);
        RobotCommand goToBall(const Vector2 &targetBallPos, TravelStrategy travelStrategy, bool ballIsFarFromTarget);

        // get status of ball handling
    public:
        enum Status : short {
          GET_BALL,
          HANDLING_BALL,
          FINALIZING,
          SUCCESS,
          FAILURE
        };
        Status getStatus();
    private:
        Status status = GET_BALL;
        void printStatus();

    public:
        explicit BallHandlePosControl(bool canMoveInDefenseArea = true);
        ~BallHandlePosControl();
        void setMaxVelocity(double maxV);
        void setMaxForwardsVelocity(double maxV);
        void setMaxBackwardsVelocity(double maxV);
        RobotCommand getRobotCommand(const RobotPtr &r, const Vector2 &targetP, const Angle &targetA) override;
        RobotCommand getRobotCommand(const RobotPtr &r, const Vector2 &targetP, const Angle &targetA,
                TravelStrategy travelStrategy);
        RobotCommand getRobotCommand(const RobotPtr &r, const Vector2 &targetP) override;

        RobotCommand goToMovingBall();
        RobotCommand goToIdleBall(const Vector2 &targetBallPos, TravelStrategy travelStrategy,
                bool ballIsFarFromTarget);
};

} //control
} //ai
} //rtt

#endif //ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H
