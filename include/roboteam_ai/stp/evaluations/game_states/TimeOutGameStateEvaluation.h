//
// Created by jordi on 28-04-20.
//

#ifndef RTT_TIMEOUTGAMESTATEEVALUATION_H
#define RTT_TIMEOUTGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {

/**
 * Invariant for the game state timeout
 */
class TimeOutGameStateEvaluation : public BaseEvaluation {
   public:
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    const char* getName() override { return "gs::TimeOut"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_TIMEOUTGAMESTATEEVALUATION_H
