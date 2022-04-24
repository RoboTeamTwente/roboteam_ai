//
// Created by john on 3/9/20.
//

#include "stp/PlayDecider.hpp"

namespace rtt::ai::stp {
Play *PlayDecider::decideNewPlay(world::World* world, const std::vector<std::unique_ptr<Play>>& plays, const std::string& interfacePlay) {
    // Score all valid plays
    auto validPlayScores = ai::stp::PlayDecider::getValidPlayScores(world, plays);

    ai::stp::Play* newPlay;
    if (SETTINGS.getUseReferee()) {
        // If we follow the referee, we pick a valid play with the highest score
        auto it = std::max_element(validPlayScores.begin(), validPlayScores.end(), [](auto& lhs, auto& rhs) { return lhs.second < rhs.second; });

        if (it == validPlayScores.end()) {
            RTT_WARNING("No valid plays found")
            // If no valid plays, pick default play
            newPlay = getPlayByName("Defend Pass", plays);
        } else {
            newPlay = (*it).first;
        }
    } else {
        // We pick the play selected by the interface
        newPlay = getPlayByName(interfacePlay, plays);
        // If that play does not exist, pick default play
        if (newPlay == nullptr) {
            RTT_WARNING("Invalid play '", interfacePlay, "' selected by the interface")
            // If invalid play selected by interface, pick halt
            newPlay = getPlayByName("Halt", plays);
        }
    }

    return newPlay;
}

std::vector<std::pair<Play*, uint8_t>> PlayDecider::getValidPlayScores(world::World* world, const std::vector<std::unique_ptr<Play>>& plays) {
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

Play *PlayDecider::getPlayByName(const std::string& name, const std::vector<std::unique_ptr<Play>>& plays) {
    auto found = std::find_if(plays.begin(), plays.end(), [&](auto& play) { return play->getName() == name; });
    if (found == plays.end()) {
        RTT_ERROR("Could not find play by name");
        return nullptr;
    }
    return found->get();
}

}  // namespace rtt::ai::stp