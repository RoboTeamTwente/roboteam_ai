//
// Created by agata on 14/01/2022.
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
     * Calculates the score of this play to determine which play is best in this situation
     * @param field The current Field
     * @return The score of this play (0-255)
     */
    uint8_t score(const rtt::world::Field& field) noexcept override;

    /**
     * Assigns robots to roles of this play
     */
    Dealer::FlagMap decideRoleFlags() const noexcept override;

    /**
     * Calculates info for the roles
     */
    void calculateInfoForRoles() noexcept override;

    /**
     * Calculate info for the roles that need to be calculated for scoring
     */
    void calculateInfoForScoredRoles(world::World*) noexcept override{};

    /**
     * Gets the play name
     */
    const char* getName() override;

   protected:
    /**
     * Calculates info for the defenders
     */
    void calculateInfoForDefenders() noexcept;

    /**
     * Calculates info for the keeper
     */
    void calculateInfoForKeeper() noexcept;

    /**
     * Calculates info for the RobotDefenders
     */
    void calculateInfoForRobotDefenders() noexcept;

    /**
     * Calculates info for the offenders
     */
    void calculateInfoForOffenders() noexcept;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_DEFENDPASS_H
