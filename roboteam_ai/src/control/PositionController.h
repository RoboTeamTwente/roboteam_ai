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
    NO_PREFERENCE,
    BALL_CONTROL,
    BASIC,
    FORCE,
    NUMERIC_TREES
};

class PositionController {

    private:
        using RobotPtr = std::shared_ptr<roboteam_msgs::WorldRobot>;
        using Command = roboteam_msgs::RobotCommand;

        PosVelAngle ballControl(RobotPtr robot, Vector2 &targetPos);
        ControlGoToPosBallControl ballControlController;

        PosVelAngle numTree(RobotPtr robot, Vector2 &targetPos);
        NumTreePosControl numTreeController;

        PosVelAngle force(RobotPtr robot, Vector2 &targetPos);
//        ControlGoToPosForce forceController;

        PosVelAngle basic(RobotPtr robot, Vector2 &targetPos);
//        ControlGoToPosBasic basicController;

        PIDController velPID;
        PIDController posPID;
        bool avoidBall = false;
        bool canGoOutsideField = false;

        PosVelAngle pidController(const RobotPtr &robot, PosVelAngle target);
        void checkInterfacePID();

    public:
        PositionController();

        void clear(PosControlType goToType);
        PosVelAngle goToPos(RobotPtr robot, Vector2 &position);
        PosVelAngle goToPos(RobotPtr robot, Vector2 &position, PosControlType goToType);

        void setAvoidBall(bool _avoidBall);
        void setCanGoOutsideField(bool _canGoOutsideField);

};

} //control
} //ai
} //rtt

#endif //ROBOTEAM_AI_POSITIONCONTROLLER_H
