//
// Created by jessevw on 06.12.19.
//

#include "analysis/play-utilities/Play.h"

#include "analysis/play-utilities/invariants/AlwaysFalseInvariant.h"
#include "analysis/play-utilities/invariants/AlwaysTrueInvariant.h"
#include "analysis/play-utilities/invariants/BallBelongsToUsInvariant.h"
#include "analysis/play-utilities/invariants/BallOnOurSideInvariant.h"

namespace rtt::ai::analysis {
    Play::Play(std::string name) : name {name} {};

    std::string_view Play::getName() { return name; }

    const std::shared_ptr<bt::BehaviorTree> &Play::getTree() const {
        return tree;
    }
}  // namespace rtt::ai::analysis