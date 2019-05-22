//
// Created by thijs on 12-4-19.
//

#ifndef ROBOTEAM_AI_HARASSROBOTCOACH_H
#define ROBOTEAM_AI_HARASSROBOTCOACH_H

#include <roboteam_utils/Vector2.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/coach/heuristics/PassScore.h>

namespace rtt {
namespace ai {
namespace coach {

class HarassRobotCoach {
    private:
        struct HarassTarget {
          Vector2 harassPosition;
          int harassRobot;
        };

        const double MIN_DISTANCE_BETWEEN_MIDFIELDERS = 2.0;
        const double DISTANCE_FROM_SIDES = 1.0;
        const double HARASS_THRESHOLD = 1.2;
        const double TOO_CLOSE_TO_BALL_DISTANCE = 0.5;
        const double MINIMUM_HARASS_VELOCITY = 1.0;

        const double DEFAULT_HARASSING_DISTANCE = 0.5;
        const double HARASSER_SECONDS_AHEAD = 0.5;

        const int GRID_SIZE = 3;
        const double GRID_INTERVAL = 0.03;

        using Robot = world::Robot;
        using RobotPtr = std::shared_ptr<Robot>;
        using Ball = world::Ball;
        using BallPtr = std::shared_ptr<Ball>;

        BallPtr ball;
        double bestXPos;
        std::vector<RobotPtr> currentMidfielders;
        std::map<int, Vector2> targetPositions;

        std::map<int, RobotPtr> targetRobotsToHarass;

        Vector2 harassRobot(const RobotPtr &thisRobot, int opponentId = - 1);

        int getRobotIdCloseToEnemyRobot(const RobotPtr &enemyRobot) const;

        Vector2 getBestReceiveLocation(const RobotPtr &thisRobot);
        HarassRobotCoach::HarassTarget findRobotToHarass(const RobotPtr &thisRobot, bool goAfterBall);
        bool robotAlreadyBeingHarassed(int myId, int opponentID);

        Vector2 standInMidField(const RobotPtr &thisRobot);
    public:

        HarassTarget getHarassPosition(const RobotPtr &thisRobot);
        Angle getHarassAngle(const RobotPtr &thisRobot);
        HarassTarget initialize(RobotPtr &thisRobot);

    void setClosestRobots(const RobotPtr &thisRobot, bool goAfterBall, double &closestRobotToBallDistance,
                          RobotPtr &closestRobotToBall, RobotPtr &closestRobotToHarasser);

    Vector2 keepDistanceBetweenHarassers(const RobotPtr &thisRobot);
};

extern HarassRobotCoach g_harassRobotCoach;

} //coach
} //ai
} //rtt

#endif //ROBOTEAM_AI_HARASSROBOTCOACH_H
