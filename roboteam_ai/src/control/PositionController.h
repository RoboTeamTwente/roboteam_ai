//
// Created by thijs on 10-12-18.
//



#ifndef ROBOTEAM_AI_CONTROLGOTOPOS_H
#define ROBOTEAM_AI_CONTROLGOTOPOS_H

#include "positionControllers/PositionControlIncludes.h"

//  ______________________
//  |                    |
//  |  INCLUDE GoToPos   |
//  |____________________|
//

#include "positionControllers/ControlGoToPosBallControl.h"
#include "roboteam_ai/src/control/positionControllers/NumTreePosControl.h"


namespace rtt {
namespace ai {
namespace control {

enum PosControlType {
    noPreference,
    ballControl,
    basic,
    force,
    numTree
};

class PositionController {

    private:
        using RobotPtr = std::shared_ptr<roboteam_msgs::WorldRobot>;
        using Command = roboteam_msgs::RobotCommand;

        PosVelAngle goToPosBallControl(RobotPtr robot, Vector2 &targetPos);
        ControlGoToPosBallControl gtpBallControl;

        PosVelAngle numTreePosControl(RobotPtr robot, Vector2 &targetPos);
        NumTreePosControl numTreeController;

        PosVelAngle goToPosForce(RobotPtr robot, Vector2 &targetPos);
//        ControlGoToPosForce forceController;

        PosVelAngle goToPosBasic(RobotPtr robot, Vector2 &targetPos);
//        ControlGoToPosBasic basicController;

        double errorMargin = 0.3;
        double distanceToTarget(RobotPtr robot, Vector2 &targetPos);

        PIDController velPID;
        PIDController posPID;
        bool PIDHasInitialized = false;

        PosVelAngle pidController(const RobotPtr &robot, PosVelAngle target);
        void initializePID();
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

#endif //ROBOTEAM_AI_CONTROLGOTOPOS_H
