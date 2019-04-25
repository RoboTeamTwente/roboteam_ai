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
        const double MIN_DISTANCE_BETWEEN_MIDFIELDERS = 2.0;
        const double DISTANCE_FROM_SIDES = 1.0;
        const double HARASS_THRESHOLD = 1.2;
        const double TOO_CLOSE_TO_BALL_DISTANCE = 0.5;
        const double MINIMUM_HARASS_VELOCITY = 1.0;

        const double DEFAULT_HARASSING_DISTANCE = 0.8;
        const double HARASSER_SECONDS_AHEAD = 0.5;

        using Robot = world::Robot;
        using RobotPtr = std::shared_ptr<Robot>;
        using Ball = world::Ball;
        using BallPtr = std::shared_ptr<Ball>;

        BallPtr ball;
        double bestXPos;
        std::vector<Vector2> currentRobotPositions;
        std::vector<Vector2> targetRobotPositions;
        std::vector<RobotPtr> targetRobotsToHarass;

        Vector2 harassRobot(int myIndex, int id = -1, bool stayInMidfield = true);
        Vector2 initialize(const Vector2 &currentLocation, int &myIndex);
        Vector2 standFree(const RobotPtr &ourRobotWithBall, const RobotPtr &thisRobot, int myIndex);
        int getRobotIndexCloseToEnemyRobot(const RobotPtr &enemyRobot) const;

        Vector2 getHarassPositionWhereTheyHaveBall(const RobotPtr &thisRobot, int &myIndex);
        Vector2 getHarassPositionWhereWeHaveBall(const RobotPtr &robotWithBall,
                const RobotPtr &thisRobot, int &myIndex);

        Vector2 standInMidField(const RobotPtr &thisRobot, int &myIndex);
    public:
        Vector2 getHarassPosition(const RobotPtr &thisRobot, int &myIndex);
        Angle getHarassAngle(const RobotPtr &thisRobot, int &myIndex);
};

extern HarassRobotCoach g_harassRobotCoach;

} //coach
} //ai
} //rtt

#endif //ROBOTEAM_AI_HARASSROBOTCOACH_H
