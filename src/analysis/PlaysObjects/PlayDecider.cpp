//
// Created by jessevw on 17.12.19.
//

#include "include/roboteam_ai/analysis/PlaysObjects/PlayDecider.h"

namespace rtt::ai::analysis {

    std::shared_ptr<rtt::ai::analysis::Play> PlayDecider::decideBestPlay(world::World *world, world::Field *field, std::vector<std::shared_ptr<rtt::ai::analysis::Play>> validPlays) {
        int max = -1;
        if (validPlays.empty()) {
            std::cerr << "validPlays array is empty!" << std::endl;
        }
        // Each play has its own internal score based on how it good it is, independent of things like our current score.
        for (auto play : validPlays) {
            int temp = play->scorePlay(world, field);
            if(temp > max) {
                max = temp;
                bestPlay = play;
            }
        }
        std::cout << max;
        return bestPlay;

    }
}
