//
// Created by jessevw on 04.12.19.
//

#ifndef RTT_PLAYCHECKER_H
#define RTT_PLAYCHECKER_H

#include "Play.h"

namespace rtt::ai::analysis {

    class PlayChecker {
    public:
        PlayChecker();

        const std::shared_ptr<Play> &getCurrentPlay() const;

        void setCurrentPlay(const std::shared_ptr<Play> &currentPlay);

        /**
         * Updates the PlayChecker. When this function is called, we check if the play is still valid for the current gamestate,
         * and if it is not new plays are calculated that are valid for the gamestate.
         */
        bool update(world::World *world, world::Field *field);

    private:
        /**
         * @brief Checks if the invariants of the current play are true for the gamestate
         * @param world the current world
         * @param field the current field
         * @return true if invariants of the play being executed are true, false otherwise
         */
        bool checkCurrentGameInvariants(rtt::ai::world::World *world, rtt::ai::world::Field *field);

        /**
         * Vector of all plays (before pruning)
         */
        std::vector<std::shared_ptr<Play>> allPlays;

        /**
         * Vector of all plays that are valid for the current world and field state
         */
        std::vector<std::shared_ptr<Play>> validPlays;

        /**
         * The play currently in execution
         */
        std::shared_ptr<Play> currentPlay;

    public:
        const std::vector<std::shared_ptr<Play>> &getValidPlays() const;

    private:
        /**
         * Determines which plays are valid by cycling through the allplays vector and seeing which plays' isValid() methods return true
         * @param world
         * @param field
         */
        void determineNewPlays(world::World *world, world::Field *field);
    };
}  // namespace rtt::ai::analysis

#endif  // RTT_PLAYCHECKER_H
