//
// Created by john on 3/9/20.
//

#ifndef RTT_ROLE_HPP
#define RTT_ROLE_HPP
#include <vector>

#include "Tactic.h"

namespace rtt::ai::stp {
    /**
     * Role class used in STP, pretty self explanatory
     * essentially a state machine of Tactics
     */
    class Role {
    public:
        /**
         * Function that's called every tick, default implementation is robotTactics.update();
         * @param info TacticInfo to be passed to update()
         * @return The status that the current tactic returns
         */
        [[nodiscard]] virtual Status update(stp::TacticInfo const& info) noexcept;

        /**
         * @return True if all tactics returned Status::finish
         */
        [[nodiscard]] bool finished() const noexcept;

    protected:
        /**
         * Robot for which this tactic is
         */
        uint8_t robotId{};

        /**
         * State machine that keeps track of tactic states
         */
        collections::state_machine<Tactic, Status, TacticInfo> robotTactics;
    };

} // namespace rtt::ai::analysis

#endif //RTT_ROLE_HPP
