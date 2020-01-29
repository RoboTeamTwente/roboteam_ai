//
// Created by jessevw on 06.12.19.
//

#include "analysis/PlaysObjects/Play.h"
#include "analysis/PlaysObjects/Invariants/AlwaysFalseInvariant.h"
#include "analysis/PlaysObjects/Invariants/AlwaysTrueInvariant.h"
#include "analysis/PlaysObjects/Invariants/BallBelongsToUsInvariant.h"
#include "analysis/PlaysObjects/Invariants/BallOnOurSideInvariant.h"

namespace rtt::ai::analysis {
Play::Play() {}

std::string Play::getName() { return name; }

Play::Play(std::string name, std::vector<std::function<bool(world::World * , world::Field * )>> invariants) {
    this->name = name;
    this->invariants = invariants;
}

bool Play::isValidPlay(rtt::ai::world::World *world, rtt::ai::world::Field *field) {
    return BallOnOurSideInvariant::isValid(world, field) && BallBelongsToUsInvariant::isValid(world, field) && AlwaysFalseInvariant::isValid(world, field) &&
        AlwaysTrueInvariant::isValid(world, field);
}

void Play::setInvariants(const std::vector<std::function<bool(world::World * , world::Field * )>> &invariants) { this->invariants = invariants; }

}  // namespace rtt::ai::analysis