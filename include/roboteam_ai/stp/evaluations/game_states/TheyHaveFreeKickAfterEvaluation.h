//
// Created by alexander on 13-07-22.
//

#ifndef RTT_THEYHAVEFREEKICKAFTEREVALUATION_H
#define RTT_THEYHAVEFREEKICKAFTEREVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {

/**
 * Invariant for the game state stop
 */
class TheyHaveFreeKickAfterEvaluation : public BaseEvaluation {
   public:
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    const char* getName() override { return "gs::TheyHaveFreeKickAfter"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_THEYHAVEFREEKICKAFTEREVALUATION_H
