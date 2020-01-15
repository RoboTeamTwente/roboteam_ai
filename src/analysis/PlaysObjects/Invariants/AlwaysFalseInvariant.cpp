//
// Created by jessevw on 11.12.19.
//

#include "include/roboteam_ai/analysis/PlaysObjects/Invariants/AlwaysFalseInvariant.h"
#include "world_old/World.h"
#include "world_old/Field.h"
#include "world_old/Ball.h"

namespace rtt::ai::analysis {
    bool AlwaysFalseInvariant::isValid(rtt::ai::world::World *world, rtt::ai::world::Field *field) {
        return false;
    }
}
