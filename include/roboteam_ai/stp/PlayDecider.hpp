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
    static inline Play *interfacePlay;

   public:
    /**
     * Sets the locked play, read variable above
     * @param play Play to lock to
     */
    static void lockInterfacePlay(Play *play);

    static bool interfacePlayChanged;

    /**
     * This function checks if there is a locked play. If there is, pick that play.
     * If there isn't, pick the play with the highest score
     * @param pWorld The world pointer
     * @param plays the vector of plays
     * @return The best play for the current tick
     * (either a locked play through the interface or just the highest scored play)
     */
    Play *decideBestPlay(const std::vector<Play *>& plays, PlayEvaluator& playEvaluator) noexcept;
};
}  // namespace rtt::ai::stp

#endif  // RTT_PLAYDECIDER_HPP
