//
// Created by jessevw on 04.12.19.
//

#include "stp/play-utilities/PlayChecker.h"

#include <stp/play-utilities/invariants/AlwaysTrueInvariant.h>

#include "stp/play-utilities/Play.h"
#include "stp/play-utilities/invariants/BallBelongsToUsInvariant.h"
#include "stp/play-utilities/invariants/BallOnOurSideInvariant.h"
#include "functional"

namespace rtt::ai::analysis {

bool PlayChecker::checkCurrentGameInvariants(rtt::ai::world::World *world, const rtt::ai::world::Field &field) { return true; }

/**
 * Determines what plays are viable given the current world, ref states and invariants/preconditions, and stores them in the validPlays vector
 * TODO: add lambda here, to make it faster and cleaner
 */
void PlayChecker::determineNewPlays(rtt::ai::world::World *world, const rtt::ai::world::Field &field) {
    validPlays.clear();
    for (auto const &play : allPlays) {
        if (play->isValidPlay(world, field)) {
            validPlays.push_back(play.get());
        }
    }
}

bool PlayChecker::update(rtt::ai::world::World *world, const rtt::ai::world::Field &field) {
    if (checkCurrentGameInvariants(world, field)) {
        //        std::cout << "current play is still valid" << std::endl;
        return true;
    }
    // Otherwise we select new plays for the playdecider by updating the validPlays vector of this object. This can then be accessed by the playdecider.
    else {
        std::cout << "current play is not valid anymore, calculating new valid plays" << std::endl;
        determineNewPlays(world, field);
        return false;
    }
}

const std::vector<Play *> PlayChecker::getValidPlays() const { return validPlays; }

}  // namespace rtt::ai::analysis
// namespace rtt::ai::analysis
