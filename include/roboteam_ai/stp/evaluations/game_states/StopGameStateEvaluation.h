//
// Created by jordi on 28-04-20.
//

#ifndef RTT_STOPGAMESTATEEVALUATION_H
#define RTT_STOPGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {

/**
 * Invariant for the game state stop
 */
class StopGameStateEvaluation : public BaseEvaluation {
   public:
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    const char* getName() override { return "gs::Stop"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_STOPGAMESTATEEVALUATION_H
