#ifndef ROBOTEAM_AI_ROTATEAROUNDROBOT_H
#define ROBOTEAM_AI_ROTATEAROUNDROBOT_H

#include <world_new/views/RobotView.hpp>
#include "control/RobotCommand.h"

namespace rtt::ai::control {

class RotateWithBall {
   public:
    explicit RotateWithBall() = default;
    RobotCommand getRobotCommand(const world_new::view::RobotView r, const Vector2 &targetP, const Angle &targetA);
};

}  // namespace rtt::ai::control

#endif  // ROBOTEAM_AI_ROTATEAROUNDROBOT_H
