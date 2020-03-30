//
// Created by timo on 3/30/20.
//

#ifndef RTT_AGGRESSIVEFORMATION_H
#define RTT_AGGRESSIVEFORMATION_H

#include <stp/Play.hpp>

namespace rtt::ai::stp::play {

class AggressiveFormation : public Play {
   public:
    /**
     * Constructor that initializes roles with roles that are necessary for this play
     */
    AggressiveFormation(std::string playName);

    bool isValidPlayToStart(world_new::World* world) noexcept override;

    bool isValidPlayToKeep(world_new::World* world) noexcept override;

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

   protected:
    bool shouldRoleSkipEndTactic() override;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_AGGRESSIVEFORMATION_H
