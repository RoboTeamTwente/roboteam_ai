//
// Created by jordi on 28-04-20.
//

#ifndef RTT_KICKOFFTHEMGAMESTATEEVALUATION_H
#define RTT_KICKOFFTHEMGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {

/**
 * Invariant for the game state kick off them
 */
class KickOffThemGameStateEvaluation : public BaseEvaluation {
   public:
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    const char* getName() override { return "gs::KickOffThem"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_KICKOFFTHEMGAMESTATEEVALUATION_H
