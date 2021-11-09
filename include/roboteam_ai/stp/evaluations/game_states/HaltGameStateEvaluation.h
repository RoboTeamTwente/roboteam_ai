//
// Created by ratoone on 27-03-20.
//

#ifndef RTT_HALTGAMESTATEEVALUATION_H
#define RTT_HALTGAMESTATEEVALUATION_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
class HaltGameStateEvaluation : public BaseEvaluation {
   public:
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    const char* getName() override { return "gs::Halt"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_HALTGAMESTATEEVALUATION_H
