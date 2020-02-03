#ifndef ROBOTEAM_AI_ROTATEAROUNDROBOT_H
#define ROBOTEAM_AI_ROTATEAROUNDROBOT_H

#include <include/roboteam_ai/world/Robot.h>
#include "control/RobotCommand.h"

namespace rtt::ai::control {

    class RotateWithBall {
        public:
        explicit RotateWithBall() = default;
        RobotCommand getRobotCommand(const std::shared_ptr<world::Robot> &r, const Vector2 &targetP, const Angle &targetA);
    };

}  // namespace rtt::ai::control

#endif  // ROBOTEAM_AI_ROTATEAROUNDROBOT_H
