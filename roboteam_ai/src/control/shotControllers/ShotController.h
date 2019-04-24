//
// Created by mrlukasbos on 24-4-19.
//

#ifndef ROBOTEAM_AI_SHOTCONTROLLER_H
#define ROBOTEAM_AI_SHOTCONTROLLER_H

#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
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
    bool useAutoGeneva;
    shotPrecision precision;
    Vector2 limitBallSpeed;
    Vector2 shotTargetPosition;

    std::pair<Vector2, int> getGenevePlaceBehindBall(world::Robot robot, Vector2 shotTarget); // the params are the position for the robot and the geneva angle
    Vector2 getPlaceBehindBall(world::Robot robot, Vector2 shotTarget); // the params are the position for the robot and the geneva angle
    Vector2 robotTargetPosition;

public:
    explicit ShotController(shotPrecision precision = MEDIUM, bool useAutoGeneva = true);
    ShotData getShotData(world::Robot robot, Vector2 shotTarget);
};

} // control
} // ai
} // rtt
#endif //ROBOTEAM_AI_SHOTCONTROLLER_H
