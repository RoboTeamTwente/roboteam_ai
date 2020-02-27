//
// Created by mrlukasbos on 24-4-19.
//

#ifndef ROBOTEAM_AI_SHOTCONTROLLER_H
#define ROBOTEAM_AI_SHOTCONTROLLER_H

#include <control/BasicPosControl.h>
#include <control/numtrees/NumTreePosControl.h>
#include "control/RobotCommand.h"
#include "gtest/gtest_prod.h"
#include "world_new/World.hpp"

namespace rtt::world_new::view {
    class RobotView;
    class BallView;
    class WorldDataView;
}

namespace rtt::ai::control {

enum ShotPrecision {
    LOW,     // not accurate but fast
    MEDIUM,  // quite accurate and quite fast
    HIGH     // very accurate but slow
};

// ball speeds
enum BallSpeed {
    DRIBBLE_KICK,

    BALL_PLACEMENT,
    PASS,
    MAX_SPEED
};

class ShotController {
    FRIEND_TEST(ShotControllerTest, it_calculates_kickforce);
    FRIEND_TEST(ShotControllerTest, it_locates_robots_properly);
    FRIEND_TEST(ShotControllerTest, it_sends_proper_shoot_commands);
    //FRIEND_TEST(ShotControllerTest, geneva_turning);

   private:
    bool init = false;
    bool isShooting = false;
    Vector2 aimTarget;

    // PID variables
    PID pid = PID(0.0, 0.0, 0.0);
    bool getPIDFromInterface = true;
    pidVals lastPid;
    void updatePid(pidVals pidValues);

    // Helpers
    Vector2 getPlaceBehindBall(const world_new::view::RobotView &robot,
                               const Vector2 &shotTarget);
    bool onLineToBall(const world_new::view::RobotView &robot, const std::pair<Vector2, Vector2> &line, ShotPrecision precision);
    bool robotAngleIsGood(world_new::view::RobotView &robot, const std::pair<Vector2, Vector2> &lineToDriveOver, ShotPrecision precision);
    double determineKickForce(double distance, BallSpeed desiredBallSpeed);

    // RobotCommand calculation
    RobotCommand
    goToPlaceBehindBall(const world::Field &field, const world_new::view::RobotView &robot,
                        const Vector2 &robotTargetPosition,
                        const std::pair<Vector2, Vector2> &driveLine,
                        world_new::view::WorldDataView &world);
    RobotCommand moveStraightToBall(world_new::view::RobotView &robot, const std::pair<Vector2, Vector2> &lineToDriveOver);
    RobotCommand shoot(RobotCommand shotData, const world_new::view::RobotView &robot, const std::pair<Vector2, Vector2> &driveLine, const Vector2 &shotTarget, bool chip,
                       BallSpeed desiredBallSpeed);
    int kickerOnTicks = 0;

   public:
    explicit ShotController() = default;
    RobotCommand
    getRobotCommand(const world::Field &field, world_new::view::RobotView &robot, const Vector2 &shotTarget,
                    bool chip, BallSpeed ballspeed, ShotPrecision precision,
                    world_new::view::BallView &ball, world_new::view::WorldDataView &world);

};

}  // namespace rtt::ai::control
#endif  // ROBOTEAM_AI_SHOTCONTROLLER_H