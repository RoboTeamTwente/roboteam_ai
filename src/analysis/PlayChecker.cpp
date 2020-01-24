//
// Created by jessevw on 04.12.19.
//

#include "analysis/PlayChecker.h"

#include <include/roboteam_ai/analysis/play-utilities/invariants/AlwaysTrueInvariant.h>

#include "analysis/play-utilities/invariants/AlwaysFalseInvariant.h"
#include "analysis/play-utilities/invariants/BallBelongsToUsInvariant.h"
#include "analysis/play-utilities/invariants/BallOnOurSideInvariant.h"
#include "analysis/play-utilities/Play.h"
#include "functional"

namespace rtt::ai::analysis {

bool PlayChecker::checkCurrentGameInvariants(rtt::ai::world::World* world, rtt::ai::world::Field* field) { return true; }

/**
 * Determines what plays are viable given the current world, ref states and invariants/preconditions, and stores them in the validPlays vector
 * TODO: add lambda here, to make it faster and cleaner
 */
void PlayChecker::determineNewPlays(rtt::ai::world::World* world, rtt::ai::world::Field* field) {
    validPlays.clear();
    for (auto play : allPlays) {
        if (play.isValidPlay(world, field)) {
            validPlays.push_back(play);
        }
    }
}

void PlayChecker::update(rtt::ai::world::World* world, rtt::ai::world::Field* field) {
    if (!checkCurrentGameInvariants(world, field)) {
        determineNewPlays(world, field);
        /// TODO: there should be a call to PlayDecider here, but this is not implemented yet.
    }
}

}  // namespace rtt::ai::analysis
