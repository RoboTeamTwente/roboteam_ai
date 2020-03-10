//
// Created by mrlukasbos on 27-3-19.
//

#ifndef ROBOTEAM_AI_POSCONTROLLER_H
#define ROBOTEAM_AI_POSCONTROLLER_H

#include <utilities/Constants.h>
#include "RobotCommand.h"
#include "world/Field.h"
#include "roboteam_utils/pid.h"

namespace rtt::world_new::view {
class WorldDataView;
class RobotView;
class BallView;
}  // namespace rtt::world_new::view

namespace rtt::ai::control {

class PosController {
   protected:
    // settings
    double customAvoidBallDistance = 0.0;
    bool customCanMoveOutOfField = true;
    bool customCanMoveInDefenseArea = true;

    // PID functions
    PID xpid = PID(0.0, 0.0, 0.0);
    PID ypid = PID(0.0, 0.0, 0.0);
    bool getPIDFromInterface = true;
    virtual void checkInterfacePID() = 0;

    RobotCommand controlWithPID(const world_new::view::RobotView robot, RobotCommand target);
    virtual Vector2 calculatePIDs(const world_new::view::RobotView robot, RobotCommand &target);

   public:
    PosController() = default;
    explicit PosController(double avoidBall, bool canMoveOutOfField, bool canMoveInDefenseArea);

    virtual RobotCommand getRobotCommand(int robotId, const Vector2 &targetPos, const Angle &targetAngle) = 0;
    virtual RobotCommand getRobotCommand(int robotId, const Vector2 &targetPos) = 0;

    bool getCanMoveOutOfField(int robotID) const;
    void setCanMoveOutOfField(bool canMoveOutOfField);
    bool getCanMoveInDefenseArea(int robotID) const;
    void setCanMoveInDefenseArea(bool canMoveInDefenseArea);
    double getAvoidBallDistance() const;
    void setAvoidBallDistance(double ballDistance = Constants::DEFAULT_BALLCOLLISION_RADIUS());
    void setAutoListenToInterface(bool listenToInterface);
    std::tuple<double, double, double> lastPid;

    void updatePid(pidVals pid);
};

}  // namespace rtt::ai::control

#endif  // ROBOTEAM_AI_POSCONTROLLER_H
