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

    bool Play::isValidPlay(rtt::ai::world::World *world, rtt::ai::world::Field *field) {
        for (auto i : invariants) {
            if(!i(world, field)) {
                return false;
            }
        }
        return true;
//        return std::all_of(invariants.cbegin(), invariants.cend(), [&](auto inv){inv(world, field);});
    }

    void Play::setInvariants(const std::vector<std::function<bool(world::World *, world::Field *)>> &invariants) {
        this->invariants = invariants;
    }




}