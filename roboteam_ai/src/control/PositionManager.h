//
// Created by thijs on 10-12-18.
//



#ifndef ROBOTEAM_AI_POSITIONCONTROLLER_H
#define ROBOTEAM_AI_POSITIONCONTROLLER_H

#include "positionControllers/ControlGoToPosBallControl.h"
#include "roboteam_ai/src/control/positionControllers/NumTreePosControl.h"

namespace rtt {
namespace ai {
namespace control {

enum PosControlType {
    BALL_CONTROL,
    BASIC,
    FORCE,
    NUMERIC_TREES
};

class PositionManager {
    private:
        using RobotPtr = std::shared_ptr<roboteam_msgs::WorldRobot>;
        using Command = roboteam_msgs::RobotCommand;

        // keep track of the current position controller
        std::shared_ptr<PosController> posController;
    public:
        PositionManager();
        PosVelAngle goToPos(RobotPtr robot, Vector2 &position);
        PosVelAngle goToPos(RobotPtr robot, Vector2 &position, PosControlType goToType);

};

} //control
} //ai
} //rtt

#endif //ROBOTEAM_AI_POSITIONCONTROLLER_H
