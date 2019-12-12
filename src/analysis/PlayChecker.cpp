//
// Created by jessevw on 04.12.19.
//

#include <include/roboteam_ai/analysis/PlaysObjects/Invariants/AlwaysTrueInvariant.h>
#include "analysis/PlaysObjects/MyPlay.h"
#include "bt/Composite.hpp"
#include "analysis/PlayChecker.h"
#include "include/roboteam_ai/analysis/PlaysObjects/Invariants/BallBelongsToUsInvariant.h"
#include "analysis/PlaysObjects/Invariants/AlwaysFalseInvariant.h"
#include "analysis/PlaysObjects/Invariants/BallOnOurSideInvariant.h"
#include "analysis/PlaysObjects/Play.h"
namespace rtt::ai::analysis {
    /**
     * I want this somewhere else but not sure yet where
     */






    PlayChecker::PlayChecker() {
        auto alwaystrueinv = new AlwaysTrueInvariant();
        auto falseinv = new AlwaysFalseInvariant();
        auto ballus = new BallBelongsToUsInvariant();
        auto onOurSide = new BallOnOurSideInvariant();

        auto AlwaysTruePlay = std::make_unique<Play>();
        auto badPlay = std::make_unique<Play>();
        auto TrueWhenBallOnOurSidePlay = std::make_unique<Play>();
        auto myPlay = std::make_unique<Play>();

        PlayChecker::allPlays = std::vector<Play> {*myPlay, *badPlay, *AlwaysTruePlay, *TrueWhenBallOnOurSidePlay};

    }

    /**
     * Object that stores the current strategy, and checks if the invariants
     * for this play are true. This object is made in the main loop
     * and will be called after the vision data is received, to determine
     * if we need to switch plays (based on whether the invariants of the current play are false).
     * If the one of the invariants is true, the PlayChecker will signal the PlayDecider to recalculate the play,
     * and give it the plays that are possible and allowed.
     */
    PlayChecker::PlayChecker(Play& play) : currentPlay {play} {
    }

    /**
     * @brief Checks if the invariants of the current play are true for the gamestate
     * @param world
     * @param field
     * @return true if invariants are true, false otherwise
     */
    bool PlayChecker::checkCurrentGameInvariants(rtt::ai::world::World* world, rtt::ai::world::Field* field) {
        std::cout << "checking if the play is still valid" << std::endl;
        return std::all_of(invariants.begin(), invariants.end(),
                [&](auto const& inv){ return inv.isTrue(world, field); });
    }



    /**
     * Determines what plays are viable given the current world, ref states and invariants/preconditions, and stores them in the validPlays vector
     * TODO: add lambda here, to make it faster and cleaner
     */
    void PlayChecker::determineNewPlays(rtt::ai::world::World* world, rtt::ai::world::Field* field) {
        validPlays = {};
        for (auto& play : allPlays) {
            if (play.isValidPlay(world, field)) {
                validPlays.push_back(play);
            }
        }
    }
    /**
     * first check if the current play is still ok
     * if not, loop through every possible play and check for each play if all of its invariants hold
     * Then add these plays to the list of allowed plays
     */
    void PlayChecker::update(rtt::ai::world::World* world, rtt::ai::world::Field* field) {
        checkCurrentGameInvariants(world, field);
        determineNewPlays(world, field);

    }
}


