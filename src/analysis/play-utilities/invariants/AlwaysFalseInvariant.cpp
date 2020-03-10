//
// Created by jessevw on 11.12.19.
//

#include <include/roboteam_ai/world_new/views/WorldDataView.hpp>
#include "include/roboteam_ai/analysis/play-utilities/invariants/AlwaysFalseInvariant.h"
#include "world/Field.h"

namespace rtt::ai::analysis {
bool AlwaysFalseInvariant::isValid(world_new::view::WorldDataView world, const world::Field &field) { return false; }
}  // namespace rtt::ai::analysis
