//
// Created by thijs on 10-12-18.
//



#ifndef ROBOTEAM_AI_CONTROLGOTOPOS_H
#define ROBOTEAM_AI_CONTROLGOTOPOS_H

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
        bool PIDHasInitialized = false;
        bool usingManualPID = false;

        PosVelAngle pidController(const RobotPtr &robot, PosVelAngle target, bool checkInterface = true);
        void initializePID();
        void initializePID(double posP, double posI, double posD, double velP = 0.0, double velI = 0.0, double velD = 0.0);
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
