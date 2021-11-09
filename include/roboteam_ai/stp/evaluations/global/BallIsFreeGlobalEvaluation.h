//
// Created by jordi on 28-04-20.
//

#ifndef RTT_BALLISFREEGLOBALEVALUATION_H
#define RTT_BALLISFREEGLOBALEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
/**
 * Check if the ball is currently not owned by any team
 */
class BallIsFreeGlobalEvaluation : public BaseEvaluation {
   public:
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    const char* getName() override { return "BallIsFree"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_BALLISFREEGLOBALEVALUATION_H
