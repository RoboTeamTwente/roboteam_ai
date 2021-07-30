//
// Created by ratoone on 27-03-20.
//

#ifndef RTT_BASEINVARIANT_H
#define RTT_BASEINVARIANT_H

#include "world/World.hpp"

namespace rtt::ai::stp::invariant {
class BaseInvariant {
   public:
    /**
     * Converts the fuzzy return to a boolean return
     * @param world the world
     * @param field the field
     * @param cutOff the cutoff number, if the truth value is higher than this number the invariant returns true
     * @return returns true when greater than cutoff and false when smaller than cutoff
     */
    [[nodiscard]] bool checkInvariant(world::view::WorldDataView world, const world::Field *field,
                                      const uint8_t cutOff = stp::control_constants::FUZZY_DEFAULT_CUTOFF) const noexcept;

    /**
     * Calculates the 'true-ness' of the invariant between 0 and 255. 0 == false, 255 == true
     * @param world the world
     * @param field the field
     * @return the 'true-ness' of this invariant during this tick
     */
    [[nodiscard]] virtual uint8_t metricCheck(world::view::WorldDataView world, const world::Field *field) const noexcept = 0;

    /**
     * dtor
     */
    virtual ~BaseInvariant() = default;

    virtual const char *getName() = 0;
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_BASEINVARIANT_H
