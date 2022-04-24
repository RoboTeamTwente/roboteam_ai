//
// Created by john on 3/9/20.
//

#ifndef RTT_PLAYDECIDER_HPP
#define RTT_PLAYDECIDER_HPP

#include "Play.hpp"

namespace rtt::ai::stp {

/**
 * Class to help pick the best play with a given world
 */
class PlayDecider {
   public:
    /**
     * Decides which new play needs to be selected
     * @param world The current world state
     * @param plays All plays from which can be decided
     * @param interfacePlay The play that is currently selected by the interface
     * @return the correct play to use, or nullptr when undecided
     */
    static Play *decideNewPlay(world::World* world, const std::vector<std::unique_ptr<Play>>& plays, const std::string& interfacePlay);

    /**
     * Gives a score to all valid plays, and returns pairs that were calculated
     * @param world The world pointer
     * @param plays All plays
     * @return pairs of valid plays with their score
     */
    static std::vector<std::pair<Play*, uint8_t>> getValidPlayScores(world::World* world, const std::vector<std::unique_ptr<Play>>& plays);

    /**
     * Gets the play that has the name. Nullptr if that play does not exist
     * @param name The name of the wanted play
     * @param plays The vector of plays in which needs to be searched
     * @return the play with the given play, or nullptr when that play does not exist
     */
    static Play *getPlayByName(const std::string& name, const std::vector<std::unique_ptr<Play>>& plays);
};
}  // namespace rtt::ai::stp

#endif  // RTT_PLAYDECIDER_HPP
