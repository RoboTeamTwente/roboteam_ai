//
// Created by jordi on 28-04-20.
//

#ifndef RTT_FREEKICKUSGAMESTATEEVALUATION_H
#define RTT_FREEKICKUSGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {

/**
 * Invariant for the game state free kick us prepare
 */
class FreeKickUsGameStateEvaluation : public BaseEvaluation {
   public:
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    const char* getName() override { return "gs::FreeKickUs"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_FREEKICKUSGAMESTATEEVALUATION_H
