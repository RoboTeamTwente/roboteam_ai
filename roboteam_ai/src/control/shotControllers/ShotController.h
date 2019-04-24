//
// Created by mrlukasbos on 24-4-19.
//

#ifndef ROBOTEAM_AI_SHOTCONTROLLER_H
#define ROBOTEAM_AI_SHOTCONTROLLER_H

#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include "ShotData.h"

namespace rtt {
namespace ai {
namespace control {

enum shotPrecision {
    LOW,
    MEDIUM,
    HIGH
};

class ShotController {
private:
    BasicPosControl basicGtp;
    NumTreePosControl numTreeGtp;

    bool useAutoGeneva;
    shotPrecision precision;
    Vector2 limitBallSpeed;
    Vector2 shotTargetPosition;

    std::pair<Vector2, int> getGenevePlaceBehindBall(world::Robot robot, Vector2 shotTarget); // the params are the position for the robot and the geneva angle
    Vector2 getPlaceBehindBall(world::Robot robot, Vector2 shotTarget); // the params are the position for the robot and the geneva angle
    Vector2 robotTargetPosition;

    ShotData goToPlaceBehindBall(world::Robot robot, Vector2 robotTargetPosition);
    ShotData moveStraightToBall(world::Robot robot);
    ShotData shoot(world::Robot robot, Vector2 shotTarget);
    double determineKickForce(double distance);

public:
    explicit ShotController(shotPrecision precision = MEDIUM, bool useAutoGeneva = true);
    ShotData getShotData(world::Robot robot, Vector2 shotTarget);
};

} // control
} // ai
} // rtt
#endif //ROBOTEAM_AI_SHOTCONTROLLER_H
