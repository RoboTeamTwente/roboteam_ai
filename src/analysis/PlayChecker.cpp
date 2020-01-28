//
// Created by jessevw on 04.12.19.
//

#include <include/roboteam_ai/analysis/PlaysObjects/Invariants/AlwaysTrueInvariant.h>
#include <include/roboteam_ai/analysis/PlaysObjects/PassAndPlayPlay.h>
#include <include/roboteam_ai/analysis/PlaysObjects/DummyPlay.h>
#include "analysis/PlayChecker.h"
#include "analysis/PlaysObjects/Invariants/BallBelongsToUsInvariant.h"
#include "analysis/PlaysObjects/Invariants/AlwaysFalseInvariant.h"
#include "analysis/PlaysObjects/Invariants/BallOnOurSideInvariant.h"
#include "analysis/PlaysObjects/Play.h"
#include "functional"


namespace rtt::ai::analysis {
    PlayChecker::PlayChecker(){
        allPlays.push_back(std::make_shared<rtt::ai::analysis::PassAndPlayPlay>("Pass and Play Playy"));
        allPlays.push_back(std::make_shared<rtt::ai::analysis::DummyPlay>("dummy"));
        currentPlay = allPlays[0];

    }


    bool PlayChecker::checkCurrentGameInvariants(rtt::ai::world::World *world, rtt::ai::world::Field *field) {
        currentPlay->isValidPlayToKeep(world, field);
    }



    void PlayChecker::determineNewPlays(rtt::ai::world::World *world, rtt::ai::world::Field *field) {
        validPlays.clear();
        for (auto play : allPlays) {
            if (play->isValidPlayToKeep(world, field)) {
                validPlays.push_back(play);
            }
        }
    }

    bool PlayChecker::update(rtt::ai::world::World* world, rtt::ai::world::Field* field) {
        // Check if the current play we are doing is still ok
        if (checkCurrentGameInvariants(world, field)) {
            std::cout << "current play is still valid" << std::endl;
            return true;
        }
        // Otherwise we select new plays for the playdecider and we send them to the playdecider
        else {
            std::cout << "current play is not valid anymore, calculating new valid plays" << std::endl;
            determineNewPlays(world, field);
            return false;
        }

    }

    const std::vector<std::shared_ptr<rtt::ai::analysis::Play>> &PlayChecker::getValidPlays() const {
        return validPlays;
    }

    void PlayChecker::setCurrentPlay(std::shared_ptr<rtt::ai::analysis::Play> newPlay) {
        currentPlay = newPlay;
    }



}


