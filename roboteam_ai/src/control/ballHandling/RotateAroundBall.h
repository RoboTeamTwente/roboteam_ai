//
// Created by thijs on 25-5-19.
//

#ifndef ROBOTEAM_AI_ROTATEAROUNDBALL_H
#define ROBOTEAM_AI_ROTATEAROUNDBALL_H

#include "../positionControllers/RobotCommand.h"

namespace rtt {
namespace ai {
namespace control {

class RotateAroundBall {
    private:
        using RobotPtr = world::Robot::RobotPtr;
        using BallPtr = world::Ball::BallPtr;
        RobotPtr robot;
        BallPtr ball;

        double maxVel = 1.3;
        double maxBallDistance = Constants::ROBOT_RADIUS()*2.0;
        double targetBallDistance = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
        Angle targetAngle;
        Vector2 targetPos;
        Vector2 previousVelocity = Vector2();
    public:
        RobotCommand getRobotCommand(const world::Robot::RobotPtr &r,
                const Vector2 &targetP, const Angle &targetA);
};

}
}
}

#endif //ROBOTEAM_AI_ROTATEAROUNDBALL_H
