//
// Created by jessevw on 06.12.19.
//

#include "analysis/PlaysObjects/Play.h"
#include "analysis/PlaysObjects/Invariants/BallOnOurSideInvariant.h"
#include "analysis/PlaysObjects/Invariants/BallBelongsToUsInvariant.h"
#include "analysis/PlaysObjects/Invariants/AlwaysFalseInvariant.h"
#include "analysis/PlaysObjects/Invariants/AlwaysTrueInvariant.h"


namespace rtt::ai::analysis {
    Play::Play(std::string name) : name{name} {}

    std::string Play::getName() {
        return name;
    }

    std::shared_ptr<bt::BehaviorTree> Play::getTree() {
        return tree;
    }




}