//
// Created by jordi on 28-04-20.
//

#ifndef RTT_FREEKICKTHEMGAMESTATEEVALUATION_H
#define RTT_FREEKICKTHEMGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {

/**
 * Invariant for the game state free kick them prepare
 */
class FreeKickThemGameStateEvaluation : public BaseEvaluation {
   public:
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    const char* getName() override { return "gs::FreeKickThem"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_FREEKICKTHEMGAMESTATEEVALUATION_H
