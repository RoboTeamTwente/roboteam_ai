//
// Created by mrlukasbos on 19-3-19.
//

#ifndef ROBOTEAM_AI_BALLPLACEMENT_H
#define ROBOTEAM_AI_BALLPLACEMENT_H

#include "PassCoach.h"
#include "roboteam_utils/Vector2.h"

namespace rtt::ai::coach {

class BallplacementCoach {
   public:
    explicit BallplacementCoach() = default;

    rtt::Vector2 getBallPlacementPos();

    rtt::Vector2 getBallPlacementBeforePos(Vector2 ballPos);

    Vector2 getBallPlacementAfterPos(const std::shared_ptr<world::Robot> &robot);
};

extern BallplacementCoach g_ballPlacement;

}  // namespace rtt::ai::coach
#endif  // ROBOTEAM_AI_BALLPLACEMENT_H
