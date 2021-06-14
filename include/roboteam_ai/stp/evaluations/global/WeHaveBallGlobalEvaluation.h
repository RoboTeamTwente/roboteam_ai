//
// Created by ratoone on 27-03-20.
//

#ifndef RTT_WEHAVEBALLGLOBALEVALUATION_H
#define RTT_WEHAVEBALLGLOBALEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
class WeHaveBallGlobalEvaluation : public BaseEvaluation {
   public:
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    const char* getName() override { return "WeHaveBall"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_WEHAVEBALLGLOBALEVALUATION_H
