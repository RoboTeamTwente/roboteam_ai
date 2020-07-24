//
// Created by john on 3/9/20.
//

#include "stp/PlayDecider.hpp"

namespace rtt::ai::stp {
Play *PlayDecider::decideBestPlay(world::World *pWorld, std::vector<Play *> plays) noexcept {
  if (lockedPlay) {
    return lockedPlay;
  }
  return *std::max_element(plays.begin(), plays.end(), [&](auto &largest, auto &play) { return largest->score(pWorld) < play->score(pWorld); });
}

void PlayDecider::lockPlay(Play *play) { lockedPlay = play; }
}  // namespace rtt::ai::stp