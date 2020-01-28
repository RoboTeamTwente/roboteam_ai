//
// Created by jessevw on 17.12.19.
//

#include "include/roboteam_ai/analysis/play-utilities/PlayDecider.h"

namespace rtt::ai::analysis {
// Maybe plays should have some sort of score based on the world how good they are
    std::shared_ptr<Play> PlayDecider::decideBestPlay(world::World *world, world::Field *field, std::vector<std::shared_ptr<Play>> validPlays) {
        int max = -1;
        if (validPlays.empty()) {
            std::cerr << "valid plays are empty" << std::endl;
        }
        for (auto const& play : validPlays) {
            int temp = play->scorePlay(world, field);
            if (temp > max) {
                max = temp;
            }
            bestPlay = play;
        }
        return bestPlay;
    }
}