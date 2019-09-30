//
// Created by mrlukasbos on 27-3-19.
//

#ifndef ROBOTEAM_AI_POSCONTROLLER_H
#define ROBOTEAM_AI_POSCONTROLLER_H

#include "RobotCommand.h"
#include <utilities/Constants.h>
#include <include/roboteam_ai/control/controllers/PidController.h>
#include <include/roboteam_ai/control/controllers/PidTwoAxesController.h>

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
        bool customCanMoveOutOfField = true;
        bool customCanMoveInDefenseArea = true;

        // PID functions
        PidTwoAxesController pid = PidTwoAxesController();
        bool getPIDFromInterface = true;
        RobotCommand controlWithPID(const RobotPtr &robot, const RobotCommand& target);
        virtual void checkInterfacePID() = 0;

    public:
        PosController() = default;
        explicit PosController(double avoidBall, bool canMoveOutOfField, bool canMoveInDefenseArea);
        virtual RobotCommand getRobotCommand(const RobotPtr &robot,
                const Vector2 &targetPos, const Angle &targetAngle) = 0;

        virtual RobotCommand getRobotCommand(const RobotPtr &robot, const Vector2 &targetPos) = 0;

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

} // control
} // ai
} // rtt

#endif //ROBOTEAM_AI_POSCONTROLLER_H
