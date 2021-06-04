//
// Created by jordi on 28-04-20.
//

#ifndef RTT_KICKOFFUSGAMESTATEEVALUATION_H
#define RTT_KICKOFFUSGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {

/**
 * Invariant for the game state kick off us
 */
class KickOffUsGameStateEvaluation : public BaseEvaluation {
   public:
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    const char* getName() override { return "gs::KickOffUs"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_KICKOFFUSGAMESTATEEVALUATION_H
