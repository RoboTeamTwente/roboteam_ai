//
// Created by thijs on 25-5-19.
//

#ifndef ROBOTEAM_AI_ROTATEAROUNDBALL_H
#define ROBOTEAM_AI_ROTATEAROUNDBALL_H

#include <include/roboteam_ai/utilities/Constants.h>
#include "control/RobotCommand.h"

namespace rtt::world_new::view{
    class RobotView;
}

namespace rtt::ai::control {

class RotateAroundBall {
   private:

    double maxVel = 1.3;
    double maxBallDistance = Constants::ROBOT_RADIUS() * 2.0;
    double targetBallDistance = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
    Angle targetAngle;
    Vector2 targetPos;
    Vector2 previousVelocity = Vector2();

   public:
    RotateAroundBall() = default;
    RobotCommand getRobotCommand(world_new::view::RobotView robot, const Vector2 &targetP, const Angle &targetA);
};

}  // namespace rtt::ai::control

#endif  // ROBOTEAM_AI_ROTATEAROUNDBALL_H
