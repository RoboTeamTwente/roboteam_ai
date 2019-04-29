//
// Created by thijs on 18-12-18.
//

#ifndef ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H
#define ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H

#include <roboteam_utils/Vector2.h>
#include "NumTreePosControl.h"
#include "BasicPosControl.h"
#include "PosController.h"

namespace rtt {
namespace ai {
namespace control {

class NumTreePosControl;
class BasicposControl;
class BallHandlePosControl : public PosController {
    private:
        double errorMargin = 0.05;
        double angleErrorMargin = 0.05;
        double maxBallDistance = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS()*3.0;
        double targetBallDistance = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
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

        PosVelAngle rotateWithBall(const RobotPtr &robot, RotateStrategy rotateStrategy);
        PosVelAngle travelWithBall(const RobotPtr &robot, TravelStrategy travelStrategy);
        void checkInterfacePID() override;

    public:
        BallHandlePosControl() = default;
        explicit BallHandlePosControl(bool canMoveInDefenseArea);

        PosVelAngle getPosVelAngle(const RobotPtr &robot, const Vector2 &target, const Angle &targetAngle) override;
        PosVelAngle getPosVelAngle(const RobotPtr &robot, const Vector2 &target) override;
};

} //control
} //ai
} //rtt

#endif //ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H
