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
    explicit PosController();
    explicit PosController(bool canMoveOutOfField, bool canMoveInDefenseArea, bool avoidBall);
    virtual PosVelAngle getPosVelAngle(RobotPtr robot, Vector2 &targetPos) = 0;
    bool getCanMoveOutOfField() const;
    void setCanMoveOutOfField(bool canMoveOutOfField);
    bool getCanMoveInDefenseArea() const;
    void setCanMoveInDefenseArea(bool canMoveInDefenseArea);
    bool getAvoidBall() const;
    void setAvoidBall(bool avoidBall);

protected:
    // settings
    bool canMoveOutOfField = false;
    bool canMoveInDefenseArea = false;
    bool avoidBall = false;

    // PID
    PIDController posPID;
    PIDController velPID;
    tuple<double, double, double> posPIDValues = {0.0, 0.0, 0.0};
    tuple<double, double, double> velPIDValues = {0.0, 0.0, 0.0};
    bool getPIDFromInterface = true;
    PosVelAngle controlWithPID(const RobotPtr &robot, PosVelAngle target);
    void checkInterfacePID();

    Vector2 calculatePIDs(const RobotPtr &robot, PosVelAngle &target);
};

} // control
} // ai
} // rtt

#endif //ROBOTEAM_AI_POSCONTROLLER_H
