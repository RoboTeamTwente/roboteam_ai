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
    Vector2 correctedBehindBallPosition;
    bool genevaIsTurning = false;
    double secondsToTurnGeneva = 1.5;
    double lastTimeGenevaChanged = 0.0;
    bool hasTargetPosition = false;

    // PositionControllers
    BasicPosControl basicGtp;
    NumTreePosControl numTreeGtp;

    // Parameters
    bool useAutoGeneva =false;
    ShotPrecision precision;
    BallSpeed ballSpeed;

    // Helpers
    std::pair<Vector2, int> getGenevaPlaceBehindBall(world::Robot robot, Vector2 shotTarget); // the params are the position for the robot and the geneva angle
    Vector2 getPlaceBehindBall(world::Robot robot, Vector2 shotTarget); // the params are the position for the robot and the geneva angle
    bool onLineToBall(const world::Robot &robot, std::pair<Vector2, Vector2> line);
    bool robotAngleIsGood(world::Robot &robot,std::pair<Vector2,Vector2> lineToDriveOver);
    double determineKickForce(double distance);

    std::pair<Vector2,Vector2> offsetLine(std::pair<Vector2,Vector2> line,int genevaState);
    // ShotData calculation
    ShotData goToPlaceBehindBall(world::Robot robot, Vector2 robotTargetPosition, std::pair<Vector2,Vector2> driveLine);
    ShotData moveStraightToBall(world::Robot robot, std::pair<Vector2, Vector2> lineToDriveOver);
    ShotData shoot(world::Robot robot,std::pair<Vector2,Vector2> driveLine, Vector2 shotTarget, bool chip);

public:
    explicit ShotController(ShotPrecision precision = MEDIUM, BallSpeed ballspeed = MAX_SPEED, bool useAutoGeneva = true);
    ShotData getShotData(world::Robot robot, Vector2 shotTarget, bool chip = false);
    void makeCommand(ShotData data, roboteam_msgs::RobotCommand &command);

    void determineGenevaAndPosition(const world::Robot &robot, const Vector2 &shotTarget, bool chip);

    void setGenevaDelay(int genevaDifference);
};

} // control
} // ai
} // rtt
#endif //ROBOTEAM_AI_SHOTCONTROLLER_H
