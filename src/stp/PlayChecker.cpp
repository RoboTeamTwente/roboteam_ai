//
// Created by john on 3/9/20.
//

#include "stp/PlayChecker.hpp"

namespace rtt::ai::stp {
std::vector<Play*> PlayChecker::getValidPlays() noexcept {
    std::vector<Play*> validPlays;

    // Only add plays that are valid
    for (auto& each : *allPlays) {
        if (each->isValidPlayToStart(world)) {
            validPlays.push_back(each.get());
        }
    }

    return validPlays;
}

void PlayChecker::update(world::World* _world) noexcept { this->world = _world; }

void PlayChecker::setPlays(std::vector<std::unique_ptr<Play>>& plays) noexcept { this->allPlays = &plays; }

Play* PlayChecker::getPlayForName(const std::string& playName) const noexcept {
    // Find play for playName param
    auto found = std::find_if(allPlays->begin(), allPlays->end(), [&](auto& play) { return play->getName() == playName; });
    if (found == allPlays->end()) {
        return nullptr;
    } else
        return found->get();
}
}  // namespace rtt::ai::stp
