//
// Created by jessevw on 06.12.19.
//

#include "include/roboteam_ai/analysis/PlaysObjects/Invariants/Invariant.h"

namespace rtt::ai::analysis {
    Invariant::Invariant() {

    }

    bool Invariant::isTrue(rtt::ai::world::World *world, rtt::ai::world::Field *field) const {
        std::cerr << "please reimplement this function in your derived class" << std::endl;
        return false;
    }


}