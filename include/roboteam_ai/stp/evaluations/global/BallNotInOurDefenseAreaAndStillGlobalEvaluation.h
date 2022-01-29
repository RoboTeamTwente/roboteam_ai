//
// Created by Alexander on 29-01-2022
//

#ifndef RTT_BALLNOTINOURDEFENSEAREAANDSTILL_H
#define RTT_BALLNOTINOURDEFENSEAREAANDSTILL_H

#include "stp/evaluations/BaseEvaluation.h"

namespace rtt::ai::stp::evaluation {
class BallNotInOurDefenseAreaAndStillGlobalEvaluation : public BaseEvaluation {
   public:
    [[nodiscard]] uint8_t metricCheck(const world::World* world, const world::Field* field) const noexcept override;

    const char* getName() override { return "BallNotInOurDefenseAreaAndStillGlobalEvaluation"; }
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_BALLNOTINOURDEFENSEAREAANDSTILL_H
