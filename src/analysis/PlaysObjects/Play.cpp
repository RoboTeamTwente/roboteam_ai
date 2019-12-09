//
// Created by jessevw on 06.12.19.
//

#include "analysis/PlaysObjects/Play.h"
namespace rtt::ai::analysis {
    /**
     * @brief Hahahahaha
     * @param invariants
     * @param inv
     */
    Play::Play(std::vector<Invariant> invariants)
        : invariants{ std::move(invariants) } {}

    Play::Play() {

    }
    const std::vector<Invariant> &Play::getInvariants() const {
        return invariants;
    }

    void Play::setInvariants(const std::vector<Invariant> &invariants) {
        this->invariants = invariants;
    }
}