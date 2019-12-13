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

    const std::vector<std::shared_ptr<Invariant>> &Play::getInvariants() const {
        return invariants;
    }

    void Play::setInvariants(const std::vector<std::shared_ptr<Invariant>> &invariants) {
        this->invariants = invariants;
    }

//    bool Play::isValidPlay(rtt::ai::world::World* world, rtt::ai::world::Field* field) {
//        return std::all_of(invariants.begin(), invariants.end(),
//                           [world, field](auto invar){ return invar->isTrue(world, field); });
//    }

    Play::Play(std::string name, std::vector<std::shared_ptr<Invariant>> invariants)
        : invariants {std::move(invariants)}, name {std::move(name)} {

    }
    std::string Play::getName() {
        return name;
    }


}