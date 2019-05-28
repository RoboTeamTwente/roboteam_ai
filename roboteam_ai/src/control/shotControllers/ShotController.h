//
// Created by mrlukasbos on 24-4-19.
//

#ifndef ROBOTEAM_AI_SHOTCONTROLLER_H
#define ROBOTEAM_AI_SHOTCONTROLLER_H

#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include <roboteam_ai/src/control/numTrees/NumTreePosControl.h>
#include "../positionControllers/RobotCommand.h"
#include "gtest/gtest_prod.h"

namespace rtt {
namespace ai {
namespace control {

enum ShotPrecision {
  LOW, // not accurate but fast
  MEDIUM, // quite accurate and quite fast
  HIGH // very accurate but slow
};

// ball speeds
enum BallSpeed {
  DRIBBLE_KICK,
  LAY_STILL_AT_POSITION,
  PASS,
  MAX_SPEED
};

class ShotController {
        FRIEND_TEST(ShotControllerTest, it_calculates_kickforce);
        FRIEND_TEST(ShotControllerTest, it_locates_robots_properly);
        FRIEND_TEST(ShotControllerTest, it_sends_proper_shoot_commands);
        FRIEND_TEST(ShotControllerTest, geneva_turning);

    private:
        bool init = false;
        bool isShooting;
        Vector2 aimTarget;
        bool genevaIsTurning = false;
        double secondsToTurnGeneva = 0.0;
        double lastTimeGenevaChanged = 0.0;

        // Helpers
        Vector2 getPlaceBehindBallForGenevaState(world::Robot robot, Vector2 shotTarget, int genevaState);
        Vector2 getPlaceBehindBall(world::Robot robot,
                Vector2 shotTarget); // the params are the position for the robot and the geneva angle
        int determineOptimalGenevaState(world::Robot robot, Vector2 shotTarget);
        bool onLineToBall(const world::Robot &robot, std::pair<Vector2, Vector2> line, ShotPrecision precision);
        bool robotAngleIsGood(world::Robot &robot, std::pair<Vector2, Vector2> lineToDriveOver,
                ShotPrecision precision);
        double determineKickForce(double distance, BallSpeed desiredBallSpeed);
        std::pair<Vector2, Vector2> shiftLineForGeneva(const std::pair<Vector2, Vector2> &line, int genevaState);

        // RobotComman calculation
        RobotCommand goToPlaceBehindBall(world::Robot robot, Vector2 robotTargetPosition,
                std::pair<Vector2, Vector2> driveLine);
        RobotCommand moveStraightToBall(world::Robot robot, std::pair<Vector2, Vector2> lineToDriveOver);
        RobotCommand shoot(world::Robot robot, std::pair<Vector2, Vector2> driveLine, Vector2 shotTarget, bool chip,
                BallSpeed desiredBallSpeed);

        RobotCommand moveAndShootGrSim(world::Robot robot, bool chip, std::pair<Vector2, Vector2> lineToDriveOver,
                BallSpeed desiredBallSpeed);
        RobotCommand moveAndShoot(world::Robot robot, bool chip, std::pair<Vector2, Vector2> lineToDriveOver,
                BallSpeed desiredBallSpeed);

    public:
        explicit ShotController() = default;
        RobotCommand getShotData(world::Robot robot, Vector2 shotTarget, bool chip = false,
                BallSpeed ballspeed = MAX_SPEED, bool useAutoGeneva = true, ShotPrecision precision = MEDIUM);
        void makeCommand(RobotCommand data, roboteam_msgs::RobotCommand &command);
        void setGenevaDelay(int genevaDifference);
};

} // control
} // ai
} // rtt
#endif //ROBOTEAM_AI_SHOTCONTROLLER_H
