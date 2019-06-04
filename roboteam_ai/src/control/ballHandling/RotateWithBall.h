//
// Created by thijs on 25-5-19.
//

#ifndef ROBOTEAM_AI_ROTATEAROUNDROBOT_H
#define ROBOTEAM_AI_ROTATEAROUNDROBOT_H

#include "../positionControllers/RobotCommand.h"

namespace rtt {
namespace ai {
namespace control {

class RotateWithBall {
        using RobotPtr = world::Robot::RobotPtr;
        using BallPtr = world::Ball::BallPtr;
        RobotPtr robot;
        BallPtr ball;

        Angle targetAngle;
        Vector2 targetPos;

    public:
        RotateWithBall() = default;
        RobotCommand getRobotCommand(world::Robot::RobotPtr r,
                const Vector2 &targetP, const Angle &targetA);
};

}
}
}

#endif //ROBOTEAM_AI_ROTATEAROUNDROBOT_H
