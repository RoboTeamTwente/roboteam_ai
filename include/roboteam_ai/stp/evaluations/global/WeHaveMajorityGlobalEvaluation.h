//
// Created by luukkn on 21-04-20.
//

#ifndef RTT_WEHAVEMAJORITYGLOBALEVALUATION_H
#define RTT_WEHAVEMAJORITYGLOBALEVALUATION_H

#include "include/roboteam_ai/stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
class WeHaveMajorityGlobalEvaluation : public BaseEvaluation {
   public:
    [[nodiscard]] uint8_t metricCheck(world::view::WorldDataView world, const world::Field* field) const noexcept override;

    const char* getName() override { return "WeHaveMajority"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_WEHAVEMAJORITYGLOBALEVALUATION_H
