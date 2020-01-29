//
// Created by jessevw on 04.12.19.
//

#include "include/roboteam_ai/analysis/play-utilities/PlayChecker.h"
#include "analysis/play-utilities/invariants/AlwaysTrueInvariant.h"
#include "analysis/play-utilities/invariants/AlwaysFalseInvariant.h"
#include "analysis/play-utilities/invariants/BallBelongsToUsInvariant.h"
#include "analysis/play-utilities/invariants/BallOnOurSideInvariant.h"
#include "analysis/play-utilities/Play.h"
#include "analysis/play-utilities/Plays/PassAndPlayPlay.h"
namespace rtt::ai::analysis {
    PlayChecker::PlayChecker() {
        currentPlay = std::make_shared<PassAndPlayPlay>("meow");
        allPlays.push_back(std::make_shared<PassAndPlayPlay>("PassAndPlay"));
    }
    bool PlayChecker::checkCurrentGameInvariants(rtt::ai::world::World *world, rtt::ai::world::Field *field) { return currentPlay->isValidPlayToKeep(world, field); }

    void PlayChecker::determineNewPlays(rtt::ai::world::World *world, rtt::ai::world::Field *field) {
        validPlays.clear();
        for (auto play : allPlays) {
            if (play->isValidPlayToStart(world, field)) {
                validPlays.push_back(play);
            }
        }
    }


    bool PlayChecker::update(rtt::ai::world::World *world, rtt::ai::world::Field *field) {
        if (checkCurrentGameInvariants(world, field)) {
            std::cout << "current play is still valid" << std::endl;
            return true;
        }
            // Otherwise we select new plays for the playdecider by updating the validPlays vector of this object. This can then be accessed by the playdecider.
        else {
            std::cout << "current play is not valid anymore, calculating new valid plays" << std::endl;
            determineNewPlays(world, field);
            return false;
        }
    }

    const std::vector<std::shared_ptr<Play>> &PlayChecker::getValidPlays() const {
        return validPlays;
    }

    const std::shared_ptr<Play> &PlayChecker::getCurrentPlay() const {
        return currentPlay;
    }

    void PlayChecker::setCurrentPlay(const std::shared_ptr<Play> &currentPlay) {
        PlayChecker::currentPlay = currentPlay;
    }
}
// namespace rtt::ai::analysis
