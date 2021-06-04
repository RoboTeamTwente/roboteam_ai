//
// Created by ratoone on 15-05-20.
//

#ifndef RTT_NORMALORFREEKICKUSGAMESTATEEVALUATION_H
#define RTT_NORMALORFREEKICKUSGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {

class NormalOrFreeKickUsGameStateEvaluation : public BaseEvaluation {
   public:
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    const char* getName() override { return "gs::NormalOrFreeKickUs"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_NORMALORFREEKICKUSGAMESTATEEVALUATION_H
