//
// Created by jessevw on 06.12.19.
//

#include "analysis/PlaysObjects/Play.h"

namespace rtt::ai::analysis {
    /**
     * @brief
     * @param invariants
     * @param inv
     */

    Play::Play() {}


    std::string Play::getName() {
        return name;
    }

    void Play::setInvariants(const std::vector<std::function<bool(world::World *, world::Field *)>> invariants) {
        this->invariants = invariants;
    }

    const std::vector<std::function<bool(world::World *, world::Field *)>> &Play::getInvariants() const {
        return invariants;
    }

    bool Play::isValidPlay(rtt::ai::world::World *world, rtt::ai::world::Field *field) const noexcept {
        return std::all_of(this->invariants.cbegin(), this->invariants.cend(),
                           [&world, &field](auto f) { return f(world, field); });
    }


}