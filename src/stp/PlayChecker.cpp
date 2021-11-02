//
// Created by john on 3/9/20.
//

#include "stp/PlayChecker.hpp"

namespace rtt::ai::stp {
std::vector<Play*> PlayChecker::getValidPlays() noexcept {
    std::vector<Play*> validPlays;

    // Only add plays that are valid
    for (auto& each : *allPlays) {
        if (each->isValidPlayToStart(playEvaluator)) {
            validPlays.push_back(each.get());
        }
    }

    return validPlays;
}

void PlayChecker::update(PlayEvaluator& _playEvaluator) noexcept {
    playEvaluator = _playEvaluator;
}

void PlayChecker::setPlays(std::vector<std::unique_ptr<Play>>& plays) noexcept { this->allPlays = &plays; }

// TODO This function should not exist, it is the result of a non working AI. Only used in ApplicationManager to hardcode a play.
Play* PlayChecker::getPlayForName(const std::string& playName) const noexcept {
    // Find play for playName param
    auto found = std::find_if(allPlays->begin(), allPlays->end(), [&](auto& play) { return play->getName() == playName; });
    if (found == allPlays->end()) {
        return nullptr;
    } else
        return found->get();
}
}  // namespace rtt::ai::stp
