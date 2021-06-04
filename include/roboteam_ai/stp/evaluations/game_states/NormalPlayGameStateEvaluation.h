//
// Created by jordi on 28-04-20.
//

#ifndef RTT_NORMALPLAYGAMESTATEEVALUATION_H
#define RTT_NORMALPLAYGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {

/**
 * Invariant for the game state normal play
 */
class NormalPlayGameStateEvaluation : public BaseEvaluation {
   public:
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    const char* getName() override { return "gs::NormalPlay"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_NORMALPLAYGAMESTATEEVALUATION_H
