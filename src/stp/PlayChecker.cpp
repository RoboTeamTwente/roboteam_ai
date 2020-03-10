//
// Created by john on 3/9/20.
//

#include "include/roboteam_ai/stp/PlayChecker.hpp"

namespace rtt::ai::stp {
    std::vector<Play*> PlayChecker::getValidPlays() noexcept {
        std::vector<Play*> validPlays;

        for (auto& each : allPlays) {
            if (each->isValidPlay(world)) {
                validPlays.push_back(each.get());
            }
        }

        return validPlays;
    }

    void PlayChecker::update(world_new::World *_world) noexcept {
        this->world = _world;
    }

    bool PlayChecker::isValid(Play *play) const noexcept {
        return play->isValidPlay(world);
    }

    void PlayChecker::setPlays(std::vector<std::unique_ptr<Play>>& plays) noexcept {
        this->allPlays = std::move(plays);
    }
} // namespace rtt::ai::stp
