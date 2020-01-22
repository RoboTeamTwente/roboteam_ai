//
// Created by jessevw on 17.12.19.
//

#ifndef RTT_PLAYDECIDER_H
#define RTT_PLAYDECIDER_H

#include <include/roboteam_ai/world/Field.h>
#include "analysis/PlaysObjects/Play.h"

namespace rtt::ai::analysis {
    class PlayDecider {
    public:
        PlayDecider() = default;
        std::shared_ptr<rtt::ai::analysis::Play> decideBestPlay(world::World* world, world::Field* field, std::vector<std::shared_ptr<rtt::ai::analysis::Play>> validPlays);

    protected:
        std::shared_ptr<rtt::ai::analysis::Play> bestPlay;
    };
}


#endif //RTT_PLAYDECIDER_H
