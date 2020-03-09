//
// Created by john on 3/9/20.
//

#include "PlayChecker.hpp"

namespace rtt::ai::stp {
    std::vector<std::reference_wrapper<Play>> rtt::ai::stp::PlayChecker::getValidPlays() noexcept {
        std::vector<std::reference_wrapper<Play>> validPlays;

        std::copy_if(allPlays.begin(), allPlays.begin(), validPlays.begin(), [&](auto& elem) {
            return elem.isValidPlay(world);
        });

        return validPlays;
    }
} // namespace rtt::ai::stp
