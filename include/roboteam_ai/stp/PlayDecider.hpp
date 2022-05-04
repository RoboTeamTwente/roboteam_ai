//
// Created by john on 3/9/20.
//

#ifndef RTT_PLAYDECIDER_HPP
#define RTT_PLAYDECIDER_HPP

#include "Play.hpp"

namespace rtt::ai::stp {

/**
 * Decides the best play from a vector of plays.
 * If there is a play set in the interface, this play will be picked.
 */
class PlayDecider {
    /**
     * play that's set from the interface in case it's overridden
     */
    static inline Play* interfacePlay;

   public:
    /**
     * Sets the locked play, read variable above
     * @param play Play to lock to
     */
    static void lockInterfacePlay(Play* play);

    /**
     * Bool indicating if the current play was manually changed in the interface
     */
    static bool interfacePlayChanged;

    /**
     * This function checks if there is a locked play. If there is, pick that play.
     * If there isn't, pick the play with the highest score
     * @param World The world pointer
     * @param plays the vector of plays
     * @return The best play for the current tick
     * (either a locked play through the interface or just the highest scored play)
     */
    static Play* decideBestPlay(const rtt::world::World* world, const std::vector<std::unique_ptr<ai::stp::Play>>& plays) noexcept;

    /**
     * Returns (a pointer to) a play with a given name
     * @param name name of the play. Should exactly match the name returned by the getName() method in the play
     * @param plays all plays to check the name for
     * @return the play with the given name, or a nullptr if no such play is found
     */
    static Play* getPlayForName(std::string name, const std::vector<std::unique_ptr<ai::stp::Play>>& plays);
};
}  // namespace rtt::ai::stp

#endif  // RTT_PLAYDECIDER_HPP
