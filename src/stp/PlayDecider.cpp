//
// Created by john on 3/9/20.
//

#include "stp/PlayDecider.hpp"

namespace rtt::ai::stp {

bool PlayDecider::interfacePlayChanged = false;

Play* PlayDecider::decideBestPlay(const rtt::world::World* world, const std::vector<std::unique_ptr<ai::stp::Play>>& plays) noexcept {
    std::vector<Play*> validPlays;
    // Only add plays that are valid
    for (auto& each : plays) {
        if (each->isValidPlayToStart()) {
            validPlays.push_back(each.get());
        }
    }
    std::vector<std::pair<Play*, uint8_t>> playsWithScores;
    playsWithScores.reserve(validPlays.size());

    auto field = world->getField().value();
    for (const auto& play : validPlays) {
        playsWithScores.emplace_back(play, play->score(field));
    }

    if (interfacePlay) {
        interfacePlayChanged = false;
        return interfacePlay;
    }

    // If there are no valid plays, default to defend pass
    if (playsWithScores.empty()) {
        RTT_WARNING("No valid plays found!");
        return getPlayForName("Defend Shot", plays);
    }

    return std::max_element(playsWithScores.begin(), playsWithScores.end(), [](auto& lhs, auto& rhs) { return lhs.second < rhs.second; })->first;
}

// This is only used by the interface to force new plays
void PlayDecider::lockInterfacePlay(Play* play) {
    PlayDecider::interfacePlayChanged = true;
    interfacePlay = play;
}

Play* PlayDecider::getPlayForName(std::string name, const std::vector<std::unique_ptr<ai::stp::Play>>& plays) {
    auto found = std::find_if(plays.begin(), plays.end(), [&](auto& play) { return play->getName() == name; });
    if (found == plays.end()) {
        RTT_ERROR("Could not find play by name");
        return nullptr;
    }
    return found->get();
}

}  // namespace rtt::ai::stp