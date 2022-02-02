//
// Created by Alexander on 28-01-2022
//

#ifndef RTT_KICKOFFUSORNORMALGAMESTATEEVALUATION_H
#define RTT_KICKOFFUSORNORMALGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {

/**
 * Invariant for the game state kick off us
 */
class KickOffUsOrNormalGameStateEvaluation : public BaseEvaluation {
   public:
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    const char* getName() override { return "gs::KickOffUsOrNormal"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_KICKOFFUSORNORMALGAMESTATEEVALUATION_H
