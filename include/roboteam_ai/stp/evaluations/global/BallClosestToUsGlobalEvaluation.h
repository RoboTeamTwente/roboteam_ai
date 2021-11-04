//
// Created by roboteam on 23/6/20.
//

#ifndef RTT_BALLCLOSESTTOUSGLOBALEVALUATION_H
#define RTT_BALLCLOSESTTOUSGLOBALEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
class BallClosestToUsGlobalEvaluation : public BaseEvaluation {
   public:
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    const char* getName() override { return "BallClosestToUs"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_BALLCLOSESTTOUSGLOBALEVALUATION_H
