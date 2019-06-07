//
// Created by mrlukasbos on 19-3-19.
//

#ifndef ROBOTEAM_AI_BALLPLACEMENT_H
#define ROBOTEAM_AI_BALLPLACEMENT_H

#include "roboteam_utils/Vector2.h"
#include "roboteam_ai/src/coach/pass/PassCoach.h"

namespace rtt {
namespace ai {
namespace coach {

class BallplacementCoach {
public:
    explicit BallplacementCoach() = default;
    rtt::Vector2 getBallPlacementPos();
    rtt::Vector2 getBallPlacementBeforePos(Vector2 ballPos);
    Vector2 getBallPlacementAfterPos(const shared_ptr<world::Robot>& robot);
};

extern BallplacementCoach g_ballPlacement;

} // coach
} // ai
} // rtt
#endif //ROBOTEAM_AI_BALLPLACEMENT_H
