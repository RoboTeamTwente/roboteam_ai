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

    int currentDesiredGeneva = -1;
    Vector2 behindBallPosition;
    bool canUpdateGeneva = true;
    double secondsToTurnGeneva = 1.5;

    ros::Time lastTimeGenevaChanged = ros::Time::now();

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
    bool onLineToBall(const world::Robot &robot, const world::World::BallPtr &ball, const Vector2 &behindBallPosition, int genevaState);
    double determineKickForce(double distance);

    // ShotData calculation
    ShotData goToPlaceBehindBall(world::Robot robot, Vector2 robotTargetPosition, int genevaState);
    ShotData moveStraightToBall(world::Robot robot, int genevaState);
    ShotData shoot(world::Robot robot, Vector2 shotTarget);

    Vector2 getGenevaLineOffsetPoint(Vector2 point, int genevaState);

public:
    explicit ShotController(ShotPrecision precision = MEDIUM, BallSpeed ballspeed = MAX_SPEED, bool useAutoGeneva = true);
    ShotData getShotData(world::Robot robot, Vector2 shotTarget);
    void makeCommand(ShotData data, roboteam_msgs::RobotCommand &command);
};

} // control
} // ai
} // rtt
#endif //ROBOTEAM_AI_SHOTCONTROLLER_H
