//
// Created by ratoone on 27-03-20.
//

#ifndef RTT_BASEEVALUATION_H
#define RTT_BASEEVALUATION_H

#include "world/World.hpp"

namespace rtt::ai::stp::evaluation {
class BaseEvaluation {
   public:
    /**
     * Calculates the 'true-ness' of the invariant between 0 and 255. 0 == false, 255 == true
     * @param world the world
     * @param field the field
     * @return the 'true-ness' of this invariant during this tick
     */
    [[nodiscard]] virtual uint8_t metricCheck(const world::World *world, const world::Field *field) const noexcept = 0;

    /**
     * dtor
     */
    virtual ~BaseEvaluation() = default;

    virtual const char *getName() = 0;
};
}  // namespace rtt::ai::stp::evaluation

#endif  // RTT_BASEEVALUATION_H
