//
// Created by jessevw on 11.12.19.
//

#include "include/roboteam_ai/analysis/PlaysObjects/Invariants/AlwaysFalseInvariant.h"

#include "world/Ball.h"
#include "world/Field.h"
#include "world/World.h"

namespace rtt::ai::analysis {
bool AlwaysFalseInvariant::isValid(rtt::ai::world::World *world, rtt::ai::world::Field *field) { return false; }
}  // namespace rtt::ai::analysis
