//
// Created by jessevw on 17.12.19.
//

#ifndef RTT_PLAYDECIDER_H
#define RTT_PLAYDECIDER_H

#include <include/roboteam_ai/world/Field.h>
#include "analysis/PlaysObjects/Play.h"

namespace rtt::ai::analysis {
class PlayDecider {
    PlayDecider() = default;
    Play decideBestPlay(world::World* world, world::Field* field, std::vector<Play> validPlays);
};
}  // namespace rtt::ai::analysis

#endif  // RTT_PLAYDECIDER_H
