//
// Created by timovdk on 4/20/20.
//

#ifndef RTT_BALLPLACEMENTTHEMGAMESTATEEVALUATION_H
#define RTT_BALLPLACEMENTTHEMGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
class BallPlacementThemGameStateEvaluation : public BaseEvaluation {
   public:
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    const char* getName() override { return "gs::BallPlacementThem"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_BALLPLACEMENTTHEMGAMESTATEEVALUATION_H
