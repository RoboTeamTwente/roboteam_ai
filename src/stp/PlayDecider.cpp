//
// Created by john on 3/9/20.
//

#include "stp/PlayDecider.hpp"

namespace rtt::ai::stp {

Play* PlayDecider::decideNewPlay(const rtt::world::World* world, const std::vector<std::unique_ptr<Play>>& plays, const std::string& interfacePlay) {
    auto validPlayScores = getValidPlayScores(world, plays);

    Play* actualInterfacePlay = getPlayByName(interfacePlay, plays);

    if (!SETTINGS.getUseReferee() && actualInterfacePlay != nullptr) {
        return actualInterfacePlay;
    }

    // If there are no valid plays, default to defend pass
    if (validPlayScores.empty()) {
        RTT_WARNING("No valid plays found!");
        return getPlayByName("Defend Pass", plays);
    }

    return std::max_element(validPlayScores.begin(), validPlayScores.end(), [](auto& lhs, auto& rhs) { return lhs.second < rhs.second; })->first;
}

std::vector<std::pair<Play*, uint8_t>> PlayDecider::getValidPlayScores(const world::World* world, const std::vector<std::unique_ptr<Play>>& plays) {
    // TODO: Combine these two for loops
    // Get all valid plays
    std::vector<ai::stp::Play*> validPlays;
    for (auto& each : plays) {
        if (each->isValidPlayToStart()) {
            validPlays.push_back(each.get());
        }
    }

    std::vector<std::pair<ai::stp::Play*, uint8_t>> validPlayScores;
    validPlayScores.reserve(validPlays.size());

    auto field = world->getField().value();
    for (const auto& play : validPlays) {
        validPlayScores.emplace_back(play, play->score(field));
    }
    return validPlayScores;
}

Play* PlayDecider::getPlayByName(std::string name, const std::vector<std::unique_ptr<ai::stp::Play>>& plays) {
    auto found = std::find_if(plays.begin(), plays.end(), [&](auto& play) { return play->getName() == name; });
    if (found == plays.end()) {
        RTT_ERROR("Could not find play by name");
        return nullptr;
    }
    return found->get();
}

}  // namespace rtt::ai::stp