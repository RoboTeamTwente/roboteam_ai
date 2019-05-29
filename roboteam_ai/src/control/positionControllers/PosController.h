//
// Created by mrlukasbos on 27-3-19.
//

#ifndef ROBOTEAM_AI_POSCONTROLLER_H
#define ROBOTEAM_AI_POSCONTROLLER_H

#include "RobotCommand.h"
#include <roboteam_ai/src/utilities/Constants.h>
#include <roboteam_ai/src/control/pid.h>

namespace rtt {
namespace ai {

namespace world {
class Robot;
class WorldData;
class Ball;
}

namespace control {

class PosController {
    protected:
        using RobotPtr = std::shared_ptr<rtt::ai::world::Robot>;
        using BallPtr = std::shared_ptr<rtt::ai::world::Ball>;
        using WorldDataPtr = std::shared_ptr<rtt::ai::world::WorldData>;

        // settings
        double customAvoidBallDistance = 0.0;
        bool customCanMoveOutOfField = false;
        bool customCanMoveInDefenseArea = false;

        // PID functions
        PID xpid = PID(0.0, 0.0, 0.0);
        PID ypid = PID(0.0, 0.0, 0.0);
        bool getPIDFromInterface = true;
        RobotCommand controlWithPID(const RobotPtr &robot, RobotCommand target);
        virtual void checkInterfacePID() = 0;

        virtual Vector2 calculatePIDs(const RobotPtr &robot, RobotCommand &target);

    public:
        PosController() = default;
        explicit PosController(double avoidBall, bool canMoveOutOfField, bool canMoveInDefenseArea);
        virtual RobotCommand getRobotCommand(const RobotPtr &robot,
                const Vector2 &targetPos, const Angle &targetAngle) = 0;

        virtual RobotCommand getRobotCommand(const RobotPtr &robot, const Vector2 &targetPos) = 0;

        bool getCanMoveOutOfField() const;
        void setCanMoveOutOfField(bool canMoveOutOfField);
        bool getCanMoveInDefenseArea() const;
        void setCanMoveInDefenseArea(bool canMoveInDefenseArea);
        double getAvoidBallDistance() const;
        void setAvoidBallDistance(double ballDistance = Constants::DEFAULT_BALLCOLLISION_RADIUS());

        std::tuple<double, double, double> lastPid;

        void updatePid(pidVals pid);
};

} // control
} // ai
} // rtt

#endif //ROBOTEAM_AI_POSCONTROLLER_H
