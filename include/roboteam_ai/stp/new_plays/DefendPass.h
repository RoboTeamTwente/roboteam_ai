//
// Created by jordi on 14-05-20.
//

#ifndef RTT_DEFENDPASS_H
#define RTT_DEFENDPASS_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

/**
 * DefendPass Play is executed when the opponent has or is close to the ball but not necessarily on our side of the field.
 * In this case the opponent most likely will pass to another robot. Our robots will namely block off robots that can
 * be passed to.
 */
class DefendPass : public Play {
   public:
    /**
     * Constructor that initializes roles with roles that are necessary for this play
     */
    DefendPass();

    /**
     * Gets the score for the current play
     *
     * On the contrary to isValidPlay() this checks how good the play actually is
     * return in range of 0 - 100
     *
     * @param world World to get the score for (world_new::World::instance())
     * @return The score, 0 - 100
     */
    uint8_t score(world_new::World* world) noexcept override;

    /**
     * Assigns robots to roles of this play
     */
    Dealer::FlagMap decideRoleFlags() const noexcept override;

    /**
     * Calculates info for the roles
     */
    void calculateInfoForRoles() noexcept override;

    /**
     * Gets the play name
     */
    const char* getName() override;

   protected:
    bool shouldRoleSkipEndTactic() override;

    /**
     * Calculates info for the defenders
     */
    void calculateInfoForDefenders() noexcept;

    /**
     * Calculates info for the blockers
     */
    void calculateInfoForBlockers() noexcept;

    /**
     * Calculates info for the keeper
     */
    void calculateInfoForKeeper() noexcept;

    /**
     * Calculates info for the harassers
     */
    void calculateInfoForHarassers() noexcept;

    /**
     * Calculates info for the offenders
     */
    void calculateInfoForOffenders() noexcept;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_DEFENDPASS_H
