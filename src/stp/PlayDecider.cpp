//
// Created by john on 3/9/20.
//

#include "stp/PlayDecider.hpp"

namespace rtt::ai::stp {

// If we cannot pick a play for any reason, these will be selected
const std::string BACKUP_REFEREE_PLAY = "Defend Pass";
const std::string BACKUP_INTERFACE_PLAY = "halt";

Play* PlayDecider::decideNewPlay(const rtt::world::World* world, const std::vector<std::unique_ptr<Play>>& plays, const std::string& interfacePlayName) {
    // In any case, give all plays a new score
    auto validPlayScores = getValidPlayScores(world, plays);

    Play* newPlay = nullptr;
    if (SETTINGS.getUseReferee()) {
        // Get the play with the highest score
        auto it = std::max_element(validPlayScores.begin(), validPlayScores.end(), [](auto& lhs, auto& rhs) { return lhs.second < rhs.second; });

        // If best play was found, use it. If not, pick the backup play
        if (it == validPlayScores.end()) {
            RTT_WARNING("Could not pick best play")
            newPlay = getPlayByName(BACKUP_REFEREE_PLAY, plays);
        } else {
            newPlay = it->first;
        }
    } else {
        // Get the play selected by the interface
        newPlay = getPlayByName(interfacePlayName, plays);

        // If the play selected by the interface could not be found, pick the backup play
        if (newPlay == nullptr) {
            RTT_WARNING("Could not find play '", interfacePlayName, "' selected by the interface")
            newPlay = getPlayByName(BACKUP_INTERFACE_PLAY, plays);
        }
    }

    if (newPlay == nullptr) RTT_ERROR("Failed to pick backup plays")
    return newPlay;
}

std::vector<std::pair<Play*, uint8_t>> PlayDecider::getValidPlayScores(const world::World* world, const std::vector<std::unique_ptr<Play>>& plays) {
    std::vector<std::pair<Play*, uint8_t>> validPlayScores;
    validPlayScores.reserve(validPlayScores.size());

    for (auto& play : plays) {
        if (play->isValidPlayToStart()) {
            validPlayScores.emplace_back(play.get(), play->score(world->getField().value()));
        }
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