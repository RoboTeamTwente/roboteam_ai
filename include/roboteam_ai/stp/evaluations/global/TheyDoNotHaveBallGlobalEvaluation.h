//
// Created by Alexander on 27-03-20.
//

#ifndef RTT_THEYDONOTHAVEBALLGLOBALEVALUATION_H
#define RTT_THEYDONOTHAVEBALLGLOBALEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
class TheyDoNotHaveBallGlobalEvaluation : public BaseEvaluation {
   public:
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    const char* getName() override { return "TheyDoNotHaveBall"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_THEYDONOTHAVEBALLGLOBALEVALUATION_H
