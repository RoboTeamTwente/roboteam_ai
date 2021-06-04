//
// Created by jordi on 28-04-20.
//

#ifndef RTT_KICKOFFUSPREPAREGAMESTATEEVALUATION_H
#define RTT_KICKOFFUSPREPAREGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {

/**
 * Invariant for the game state kick off us prepare
 */
class KickOffUsPrepareGameStateEvaluation : public BaseEvaluation {
   public:
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    const char* getName() override { return "gs::KickOffUsPrepare"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_KICKOFFUSPREPAREGAMESTATEEVALUATION_H
