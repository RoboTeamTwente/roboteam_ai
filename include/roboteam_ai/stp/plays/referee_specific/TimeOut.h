//
// Created by jordi on 01-05-20.
//

#ifndef RTT_TIMEOUT_H
#define RTT_TIMEOUT_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

class TimeOut : public Play {
   public:
    /**
     * Constructor that initializes roles with roles that are necessary for this play
     */
    TimeOut();

    /**
     * Calculates the score of this play to determine which play is best in this situation
     * @param field The current Field
     * @return The score of this play (0-255)
     */
    uint8_t score(const rtt::world::Field &field) noexcept override;

    /**
     * Assigns robots to roles of this play
     */
    Dealer::FlagMap decideRoleFlags() const noexcept override;

    /**
     * Info that should be calculated but is not necessary for scoring
     */
    void calculateInfoForRoles() noexcept override;

    /**
     * Info that should be calculated for scoring of the play
     */
    void calculateInfoForScoredRoles(world::World *) noexcept override{};

    /**
     * Gets the play name
     */
    const char *getName() override;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_TIMEOUT_H
