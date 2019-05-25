//
// Created by thijs on 25-5-19.
//

#ifndef ROBOTEAM_AI_ROTATEAROUNDROBOT_H
#define ROBOTEAM_AI_ROTATEAROUNDROBOT_H

namespace rtt {
namespace ai {
namespace control {

class DribbleBackwards;
class DribbleForwards;
class RotateAroundBall;
class RotateAroundRobot {
        using RobotPtr = world::Robot::RobotPtr;
        using BallPtr = world::Ball::BallPtr;
        RobotPtr robot;
        BallPtr ball;

        Angle targetAngle;
        Vector2 targetPos;

    public:
        RobotCommand getRobotCommand(const world::Robot::RobotPtr &r,
                const Vector2 &targetP, const Angle &targetA);
};

}
}
}

#endif //ROBOTEAM_AI_ROTATEAROUNDROBOT_H
