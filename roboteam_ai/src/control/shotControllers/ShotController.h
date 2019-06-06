//
// Created by mrlukasbos on 24-4-19.
//

#ifndef ROBOTEAM_AI_SHOTCONTROLLER_H
#define ROBOTEAM_AI_SHOTCONTROLLER_H

#include <roboteam_ai/src/control/BasicPosControl.h>
#include <roboteam_ai/src/control/numTrees/NumTreePosControl.h>
#include "roboteam_ai/src/control/RobotCommand.h"
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
        bool isShooting = false;
        Vector2 aimTarget;
        bool genevaIsTurning = false;
        double secondsToTurnGeneva = 0.0;
        double lastTimeGenevaChanged = 0.0;

        // Helpers
        Vector2 getPlaceBehindBallForGenevaState(const world::Robot& robot, const Vector2& shotTarget, int genevaState);
        Vector2 getPlaceBehindBall(const world::Robot& robot,
                const Vector2& shotTarget); // the params are the position for the robot and the geneva angle
        int determineOptimalGenevaState(const world::Robot& robot, const Vector2& shotTarget);
        bool onLineToBall(const world::Robot &robot, const std::pair<Vector2, Vector2>& line, ShotPrecision precision);
        bool robotAngleIsGood(world::Robot &robot, const std::pair<Vector2, Vector2>& lineToDriveOver,
                ShotPrecision precision);
        double determineKickForce(double distance, BallSpeed desiredBallSpeed);
        std::pair<Vector2, Vector2> shiftLineForGeneva(const std::pair<Vector2, Vector2> &line, int genevaState);

        // RobotCommand calculation
        RobotCommand goToPlaceBehindBall(world::Robot robot, const Vector2& robotTargetPosition,
                const std::pair<Vector2, Vector2>& driveLine);
        RobotCommand moveStraightToBall(world::Robot robot, const std::pair<Vector2, Vector2>& lineToDriveOver);
        RobotCommand shoot(world::Robot robot, const std::pair<Vector2, Vector2>& driveLine, const Vector2& shotTarget, bool chip,
                BallSpeed desiredBallSpeed);

        RobotCommand shootWithoutBallSensor(const world::Robot& robot, const std::pair<Vector2, Vector2>& driveLine, const Vector2& shotTarget, bool chip,
                                            BallSpeed desiredBallSpeed);


    public:
        explicit ShotController() = default;
        RobotCommand getRobotCommand(world::Robot robot, const Vector2 &shotTarget, bool chip = false,
                BallSpeed ballspeed = MAX_SPEED, bool useAutoGeneva = true, ShotPrecision precision = MEDIUM);
        void setGenevaDelay(int genevaDifference);
};

} // control
} // ai
} // rtt
#endif //ROBOTEAM_AI_SHOTCONTROLLER_H
