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

        const double errorMargin = 0.05;
        const double angleErrorMargin = 0.05;
        const double maxBallDistance = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS()*3.0;
        const double targetBallDistance = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
        const double maxForwardsVelocity = 0.5;
        const double maxBackwardsVelocity = 0.5;
        const double maxAngularVelocity = 0.2;
        bool canMoveInDefenseArea = false;

        BallPtr ball;

        Vector2 targetPos;
        Angle targetAngle;

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

        RobotCommand rotateWithBall(const RobotPtr &robot, RotateStrategy rotateStrategy);
        RobotCommand travelWithBall(const RobotPtr &robot, TravelStrategy travelStrategy);

    public:
        explicit BallHandlePosControl(bool canMoveInDefenseArea = false);

        RobotCommand getPosVelAngle(const RobotPtr &robot, const Vector2 &target, const Angle &targetAngle);
};

} //control
} //ai
} //rtt

#endif //ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H
