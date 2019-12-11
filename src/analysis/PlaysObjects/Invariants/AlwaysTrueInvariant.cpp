//
// Created by jessevw on 11.12.19.
//

#include "include/roboteam_ai/analysis/PlaysObjects/Invariants/AlwaysTrueInvariant.h"
#include "world/World.h"
#include "world/Field.h"
#include "world/Ball.h"
namespace rtt::ai::analysis {
    bool AlwaysTrueInvariant::isTrue(rtt::ai::world::World* world, rtt::ai::world::Field* field) {
        return true;
    }
}
