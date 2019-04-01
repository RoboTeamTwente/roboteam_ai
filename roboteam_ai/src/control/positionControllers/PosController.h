//
// Created by mrlukasbos on 27-3-19.
//

#ifndef ROBOTEAM_AI_POSCONTROLLER_H
#define ROBOTEAM_AI_POSCONTROLLER_H

#include "PosVelAngle.h"
#include <roboteam_ai/src/utilities/Constants.h>
#include <roboteam_ai/src/control/pid.h>
#include "roboteam_msgs/WorldRobot.h"

namespace rtt {
namespace ai {
namespace control {
    class PosController {
public:
    using RobotPtr = std::shared_ptr<roboteam_msgs::WorldRobot>;
    explicit PosController() = default;
    explicit PosController(bool avoidBall, bool canMoveOutOfField, bool canMoveInDefenseArea);
    virtual PosVelAngle getPosVelAngle(RobotPtr robot, Vector2 &targetPos) = 0;
    bool getCanMoveOutOfField() const;
    void setCanMoveOutOfField(bool canMoveOutOfField);
    bool getCanMoveInDefenseArea() const;
    void setCanMoveInDefenseArea(bool canMoveInDefenseArea);
    bool getAvoidBall() const;
    void setAvoidBall(bool avoidBall);

protected:
    // settings
    bool avoidBall = false;
    bool canMoveOutOfField = false;
    bool canMoveInDefenseArea = false;

    PID xpid = PID(2.4, 0, 0.0);
    PID ypid = PID(2.4, 0, 0.0);

    bool getPIDFromInterface = false;
    PosVelAngle controlWithPID(const RobotPtr &robot, PosVelAngle target);
    void checkInterfacePID();

    virtual Vector2 calculatePIDs(const RobotPtr &robot, PosVelAngle &target);
};

} // control
} // ai
} // rtt

#endif //ROBOTEAM_AI_POSCONTROLLER_H
