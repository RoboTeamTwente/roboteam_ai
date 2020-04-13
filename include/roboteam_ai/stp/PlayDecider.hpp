//
// Created by john on 3/9/20.
//

#ifndef RTT_PLAYDECIDER_HPP
#define RTT_PLAYDECIDER_HPP

#include "Play.hpp"

namespace rtt::ai::stp {

    class PlayDecider {
        // play that's set from the interface in case it's overridden
        static inline Play *lockedPlay;

    public:
        /**
         * Sets the locked play, read variable above
         * @param play Play to lock to
         */
        static void lockPlay(Play *play);

        Play *decideBestPlay(world_new::World *pWorld, std::vector<Play *> plays) noexcept;
    };

}  // namespace rtt::ai::stp

#endif  // RTT_PLAYDECIDER_HPP
