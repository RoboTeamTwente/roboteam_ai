//
// Created by jessevw on 11.12.19.
//

#include "include/roboteam_ai/analysis/PlaysObjects/Invariants/AlwaysTrueInvariant.h"
#include "world_old/World.h"
#include "world_old/Field.h"
#include "world_old/Ball.h"

namespace rtt::ai::analysis {
    bool AlwaysTrueInvariant::isValid(rtt::ai::world::World *world, rtt::ai::world::Field *field) {
        return true;
    };

}
