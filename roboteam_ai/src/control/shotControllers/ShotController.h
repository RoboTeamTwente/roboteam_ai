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
    bool init = false;
    bool isShooting;
    Vector2 behindBallPosition;
    Vector2 aimTarget;
    bool genevaIsTurning = false;
    double secondsToTurnGeneva = 1.5;
    double lastTimeGenevaChanged = 0.0;

    // Helpers
    Vector2 getPlaceBehindBallForGenevaState(world::Robot robot, Vector2 shotTarget, int genevaState);
    Vector2 getPlaceBehindBall(world::Robot robot, Vector2 shotTarget); // the params are the position for the robot and the geneva angle
    int determineOptimalGenevaState(world::Robot robot, Vector2 shotTarget);
    bool onLineToBall(const world::Robot &robot, std::pair<Vector2, Vector2> line, ShotPrecision precision);
    bool robotAngleIsGood(world::Robot &robot,std::pair<Vector2,Vector2> lineToDriveOver, ShotPrecision precision);
    double determineKickForce(double distance,  BallSpeed desiredBallSpeed);

    // ShotData calculation
    ShotData goToPlaceBehindBall(world::Robot robot, Vector2 robotTargetPosition, std::pair<Vector2,Vector2> driveLine);
    ShotData moveStraightToBall(world::Robot robot, std::pair<Vector2, Vector2> lineToDriveOver);
    ShotData shoot(world::Robot robot,std::pair<Vector2,Vector2> driveLine, Vector2 shotTarget, bool chip, BallSpeed desiredBallSpeed);

    ShotData moveAndShootSimulator(world::Robot robot, bool chip,std::pair<Vector2,Vector2> lineToDriveOver,BallSpeed desiredBallSpeed);
    ShotData moveAndShootReal(world::Robot robot, bool chip,std::pair<Vector2,Vector2> lineToDriveOver,BallSpeed desiredBallSpeed);

public:
    explicit ShotController() = default;
    ShotData getShotData(world::Robot robot, Vector2 shotTarget, bool chip = false, BallSpeed ballspeed = MAX_SPEED,  bool useAutoGeneva = true, ShotPrecision precision = MEDIUM);
    void makeCommand(ShotData data, roboteam_msgs::RobotCommand &command);
    void setGenevaDelay(int genevaDifference);
};

} // control
} // ai
} // rtt
#endif //ROBOTEAM_AI_SHOTCONTROLLER_H
