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

#include "positionControllers/ControlGoToPosLuTh.h"
#include "positionControllers/ControlGoToPosBallControl.h"
#include "roboteam_ai/src/control/positionControllers/NumTreePosControl.h"


namespace rtt {
namespace ai {
namespace control {

enum GoToType {
    noPreference,
    ballControl,
    basic,
    force,
    luTh_OLD,
    numTree
};

class ControlGoToPos {

    private:
        using RobotPtr = std::shared_ptr<roboteam_msgs::WorldRobot>;
        using Command = roboteam_msgs::RobotCommand;

        Vector2 goToPosBallControl(RobotPtr robot, Vector2 &targetPos);
        ControlGoToPosBallControl gtpBallControl;

        Vector2 goToPosLuTh(RobotPtr robot, Vector2 &targetPos);
        ControlGoToPosLuTh gtpLuth;

        Vector2 goToPosClean(RobotPtr robot, Vector2 &targetPos);
        NumTreePosControl numTreeController;

        Vector2 goToPosForce(RobotPtr robot, Vector2 &targetPos);
//        ControlGoToPosForce forceController;

        Vector2 goToPosBasic(RobotPtr robot, Vector2 &targetPos);
//        ControlGoToPosBasic basicController;

        double errorMargin = 0.3;
        double distanceToTarget(RobotPtr robot, Vector2 &targetPos);

        PIDController velPID;
        PIDController posPID;
        bool PIDHasInitialized = false;

        Vector2 pidController(RobotPtr robot, PosVelAngle target);
        void initializePID();
        void checkInterfacePID();

    public:
        ControlGoToPos();

        void clear(GoToType goToType);
        Vector2 goToPos(RobotPtr robot, Vector2 &position);
        Vector2 goToPos(RobotPtr robot, Vector2 &position, GoToType goToType);

        void setAvoidBall(bool _avoidBall);
        void setCanGoOutsideField(bool _canGoOutsideField);

};

} //control
} //ai
} //rtt

#endif //ROBOTEAM_AI_CONTROLGOTOPOS_H
