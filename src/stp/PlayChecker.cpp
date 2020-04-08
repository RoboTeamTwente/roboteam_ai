//
// Created by john on 3/9/20.
//

#include "include/roboteam_ai/stp/PlayChecker.hpp"

namespace rtt::ai::stp {
std::vector<Play*> PlayChecker::getValidPlays() noexcept {
    std::vector<Play*> validPlays;

    for (auto& each : *allPlays) {
        if (each->isValidPlayToStart(world)) {
            validPlays.push_back(each.get());
        }
    }

    return validPlays;
}

void PlayChecker::update(world_new::World* world) noexcept { this->world = world; }

void PlayChecker::setPlays(std::vector<std::unique_ptr<Play>>& plays) noexcept { this->allPlays = &plays; }
}  // namespace rtt::ai::stp
