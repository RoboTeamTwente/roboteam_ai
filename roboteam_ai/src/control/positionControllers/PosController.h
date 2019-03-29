//
// Created by mrlukasbos on 27-3-19.
//

#ifndef ROBOTEAM_AI_POSCONTROLLER_H
#define ROBOTEAM_AI_POSCONTROLLER_H

#include <roboteam_ai/src/control/PIDController.h>
#include "PosVelAngle.h"
#include "../../world/World.h"
#include "../../world/WorldData.h"

namespace rtt {
namespace ai {
namespace control {
class PosController {
    public:
        using Robot = world::Robot;
        using Ball = world::Ball;
        using WorldData = world::WorldData;
        using RobotPtr = std::shared_ptr<Robot>;
        using BallPtr = std::shared_ptr<Ball>;
        using WorldDataPtr = std::shared_ptr<WorldData>;

        explicit PosController();
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
        tuple<double, double, double> posPIDValues = std::tuple<double, double, double>(0.0, 0.0, 0.0);
        tuple<double, double, double> velPIDValues = std::tuple<double, double, double>(0.0, 0.0, 0.0);
        bool getPIDFromInterface = true;
        PosVelAngle controlWithPID(const RobotPtr &robot, PosVelAngle target);
        void checkInterfacePID();

        Vector2 calculatePIDs(const RobotPtr &robot, PosVelAngle &target);
};

} // control
} // ai
} // rtt

#endif //ROBOTEAM_AI_POSCONTROLLER_H
