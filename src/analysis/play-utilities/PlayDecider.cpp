//
// Created by jessevw on 17.12.19.
//

#include "include/roboteam_ai/analysis/play-utilities/PlayDecider.h"

namespace rtt::ai::analysis {
// Maybe plays should have some sort of score based on the world how good they are
    Play PlayDecider::decideBestPlay(world::World *world, world::Field *field, std::vector<Play> validPlays) { return Play(); }
}  // namespace rtt::ai::analysis