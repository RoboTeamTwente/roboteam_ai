//
// Created by jessevw on 17.12.19.
//

#include "include/roboteam_ai/analysis/play-utilities/PlayDecider.h"

namespace rtt::ai::analysis {
// Maybe plays should have some sort of score based on the world how good they are
    Play*
    PlayDecider::decideBestPlay(world::World *world, const world::Field& field, std::vector<Play*> const& validPlays) {
        int max = -1;
        if (validPlays.empty()) {
            std::cerr << "valid plays are empty" << std::endl;
        }
        for (auto & play : validPlays) {
            int temp = play->scorePlay(world, field);
            if (temp > max) {
                max = temp;
            }
            bestPlay = play;
        }
        return bestPlay;
    }
}