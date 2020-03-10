//
// Created by john on 3/9/20.
//

#ifndef RTT_PLAYDECIDER_HPP
#define RTT_PLAYDECIDER_HPP

#include "Play.hpp"

namespace rtt::ai::stp {

    class PlayDecider {
    public:
        Play *decideBestPlay(world_new::World *pWorld, std::vector<Play*> plays) noexcept;
    };

} // namespace rtt::ai::stp

#endif //RTT_PLAYDECIDER_HPP
