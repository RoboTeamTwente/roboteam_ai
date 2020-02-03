//
// Created by jessevw on 04.12.19.
//

#ifndef RTT_PLAYCHECKER_H
#define RTT_PLAYCHECKER_H

#include "analysis/play-utilities/Play.h"

namespace rtt::ai::analysis {
using namespace rtt::ai::world;

class PlayChecker {
   public:
    PlayChecker() = default;
    /**
     * Updates the PlayChecker. When this function is called, we check if the play is still valid for the current gamestate,
     * and if it is not new plays are calculated that are valid for the gamestate.
     */
    void update(world::World *world, const Field *field);

   private:
    /**
     * @brief Checks if the invariants of the current play are true for the gamestate
     * @param world the current world
     * @param field the current field
     * @return true if invariants of the play being executed are true, false otherwise
     */
    bool checkCurrentGameInvariants(rtt::ai::world::World *world, const Field *field);

    /**
     * Vector of all plays (before pruning)
     */
    std::vector<Play> allPlays;

    /**
     * Vector of all plays that are valid for the current world and field state
     */
    std::vector<Play> validPlays;

    /// TODO: implement this function. Not a priority right now
    bool checkStrategyPreconditions();

    /**
     * Determines which plays are valid by cycling through the allplays vector and seeing which plays' isValid() methods return true
     * @param world
     * @param field
     */
    void determineNewPlays(world::World *world, const Field *field);
};
}  // namespace rtt::ai::analysis

#endif  // RTT_PLAYCHECKER_H
