//
// Created by mrlukasbos on 24-4-19.
//

#ifndef ROBOTEAM_AI_SHOTCONTROLLER_H
#define ROBOTEAM_AI_SHOTCONTROLLER_H

#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include "ShotData.h"
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
    FRIEND_TEST(ShotControllerTest, it_generates_proper_shots);
private:

    // PositionControllers
    BasicPosControl basicGtp;
    NumTreePosControl numTreeGtp;

    // Parameters
    bool useAutoGeneva;
    ShotPrecision precision;
    BallSpeed ballSpeed;

    // Helpers
    std::pair<Vector2, int> getGenevaPlaceBehindBall(world::Robot robot, Vector2 shotTarget); // the params are the position for the robot and the geneva angle
    Vector2 getPlaceBehindBall(world::Robot robot, Vector2 shotTarget); // the params are the position for the robot and the geneva angle
    Vector2 robotTargetPosition;
    bool onLineToBall(const world::Robot &robot, const world::World::BallPtr &ball, const Vector2 &behindBallPosition) const;
    double determineKickForce(double distance);

    // ShotData calculation
    ShotData goToPlaceBehindBall(world::Robot robot, Vector2 robotTargetPosition);
    ShotData moveStraightToBall(world::Robot robot);
    ShotData shoot(world::Robot robot, Vector2 shotTarget);

public:
    explicit ShotController(ShotPrecision precision = MEDIUM, BallSpeed ballspeed = MAX_SPEED, bool useAutoGeneva = true);
    ShotData getShotData(world::Robot robot, Vector2 shotTarget);
    roboteam_msgs::RobotCommand makeCommand(ShotData data);
};

} // control
} // ai
} // rtt
#endif //ROBOTEAM_AI_SHOTCONTROLLER_H
