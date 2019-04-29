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
class BasicposControl;
class BallHandlePosControl {
    private:
        using BallPtr = std::shared_ptr<world::Ball>;
        using RobotPtr = std::shared_ptr<world::Robot>;

        const double errorMargin = 0.05;
        const double angleErrorMargin = 0.05;
        const double maxBallDistance = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS()*3.0;
        const double targetBallDistance = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
        bool canMoveInDefenseArea = false;

        BallPtr ball;

        Vector2 targetPos;
        Angle targetAngle;

        std::shared_ptr<NumTreePosControl> numTreePosController;
        std::shared_ptr<BasicPosControl> basicPosController;
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
