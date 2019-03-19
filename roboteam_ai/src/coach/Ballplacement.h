//
// Created by mrlukasbos on 19-3-19.
//

#ifndef ROBOTEAM_AI_BALLPLACEMENT_H
#define ROBOTEAM_AI_BALLPLACEMENT_H

#include "roboteam_utils/Vector2.h"

namespace rtt {
namespace ai {
namespace coach {

class Ballplacement {
public:
    rtt::Vector2 getBallPlacementPos();
    rtt::Vector2 getBallPlacementBeforePos(Vector2 ballPos);
    rtt::Vector2 getBallPlacementAfterPos(double RobotAngle);
};

extern Ballplacement g_ballPlacement;

} // coach
} // ai
} // rtt
#endif //ROBOTEAM_AI_BALLPLACEMENT_H
