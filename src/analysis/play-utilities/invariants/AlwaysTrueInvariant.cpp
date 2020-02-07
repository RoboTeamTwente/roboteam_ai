//
// Created by jessevw on 11.12.19.
//

#include "include/roboteam_ai/analysis/play-utilities/invariants/AlwaysTrueInvariant.h"

#include "world/Ball.h"
#include "world/Field.h"
#include "world/World.h"

namespace rtt::ai::analysis {
using namespace rtt::ai::world;

bool AlwaysTrueInvariant::isValid(rtt::ai::world::World *world, const Field *field) { return true; };

}  // namespace rtt::ai::analysis
