//
// Created by john on 3/9/20.
//

#include "include/roboteam_ai/stp/PlayDecider.hpp"

namespace rtt::ai::stp {
    Play *PlayDecider::decideBestPlay(world_new::World *pWorld, std::vector<Play*> plays) noexcept {
        return *std::max_element(plays.begin(), plays.end(), [&](auto& largest, auto& play) {
            return largest->score(pWorld) < play->score(pWorld);
        });
    }
} // namespace rtt::ai::stp