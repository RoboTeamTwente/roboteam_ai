//
// Created by mrlukasbos on 27-3-19.
//

#ifndef ROBOTEAM_AI_POSCONTROLLER_H
#define ROBOTEAM_AI_POSCONTROLLER_H

#include <roboteam_ai/src/control/PIDController.h>
#include "PosVelAngle.h"

namespace rtt {
namespace ai {
namespace control {

class PosController {
public:
    using RobotPtr = std::shared_ptr<roboteam_msgs::WorldRobot>;
    explicit PosController() = default;
    PIDController PID;
    virtual PosVelAngle getPosVelAngle(RobotPtr robot, Vector2 &targetPos) = 0;

protected:
    bool canMoveOutOfField = false;
    bool canMoveInDefenseArea = false;
    bool avoidBall = false;
};

} // control
} // ai
} // rtt

#endif //ROBOTEAM_AI_POSCONTROLLER_H
