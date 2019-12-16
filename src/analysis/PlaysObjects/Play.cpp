//
// Created by jessevw on 06.12.19.
//

#include <functional>
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

    Play::Play(std::string name, std::vector<std::function<bool(world::World *, world::Field *)>> invariants) {
        this->name = name;
        this->invariants = invariants;
    }

    void Play::setInvariants(const std::vector<std::function<bool(world::World *, world::Field *)>> &invariants) {
        this->invariants = invariants;
    }

    const std::vector<std::function<bool(world::World *, world::Field *)>> &Play::getInvariants() const {
        return invariants;
    }


}