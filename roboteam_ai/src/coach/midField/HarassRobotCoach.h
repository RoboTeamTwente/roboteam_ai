//
// Created by thijs on 12-4-19.
//

#ifndef ROBOTEAM_AI_HARASSROBOTCOACH_H
#define ROBOTEAM_AI_HARASSROBOTCOACH_H

#include <roboteam_utils/Vector2.h>
#include <roboteam_ai/src/control/ControlUtils.h>

namespace rtt {
namespace ai {
namespace coach {

class HarassRobotCoach {
    private:
        using Robot = world::Robot;
        using RobotPtr = std::shared_ptr<Robot>;
        using Ball = world::Ball;
        using BallPtr = std::shared_ptr<Ball>;

        static double bestXPos;
        static std::vector<Vector2> currentRobotPositions;
        static std::vector<Vector2> targetRobotPositions;
        static std::vector<RobotPtr> targetRobotsToHarass;

        Vector2 harassRobot(int myIndex, int id = -1);
    public:
        Vector2 getHarassPosition(const Vector2 &currentLocation, int &myIndex);
        Vector2 standFree();
};

extern HarassRobotCoach g_harassRobotCoach;

} //coach
} //ai
} //rtt

#endif //ROBOTEAM_AI_HARASSROBOTCOACH_H
