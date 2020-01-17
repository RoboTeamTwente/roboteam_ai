//
// Created by thijs on 25-5-19.
//

#ifndef ROBOTEAM_AI_ROTATEAROUNDBALL_H
#define ROBOTEAM_AI_ROTATEAROUNDBALL_H

#include <include/roboteam_ai/utilities/Constants.h>
#include "control/RobotCommand.h"

namespace rtt::ai {

namespace world {
class Robot;
class Ball;
}  // namespace world

namespace control {

class RotateAroundBall {
   private:
    using RobotPtr = std::shared_ptr<world::Robot>;
    using BallPtr = std::shared_ptr<world::Ball>;

    double maxVel = 1.3;
    double maxBallDistance = Constants::ROBOT_RADIUS() * 2.0;
    double targetBallDistance = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
    Angle targetAngle;
    Vector2 targetPos;
    Vector2 previousVelocity = Vector2();

   public:
    RotateAroundBall() = default;
    RobotCommand getRobotCommand(RobotPtr r, const Vector2 &targetP, const Angle &targetA);
};

}  // namespace control
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_ROTATEAROUNDBALL_H
