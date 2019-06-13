#ifndef ROBOTEAM_AI_ROTATEAROUNDROBOT_H
#define ROBOTEAM_AI_ROTATEAROUNDROBOT_H

#include "../../control/RobotCommand.h"

namespace rtt {
namespace ai {
namespace control {

class RotateWithBall {
public:
    explicit RotateWithBall() = default;
    RobotCommand getRobotCommand(const std::shared_ptr<world::Robot> &r, const Vector2 &targetP, const Angle &targetA);
};

}
}
}

#endif //ROBOTEAM_AI_ROTATEAROUNDROBOT_H
