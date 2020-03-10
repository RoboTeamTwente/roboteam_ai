//
// Created by jessevw on 17.12.19.
//

#ifndef RTT_PLAYDECIDER_H
#define RTT_PLAYDECIDER_H

#include <include/roboteam_ai/world/Field.h>
#include "Play.h"

namespace rtt::ai::analysis {
class PlayDecider {
   public:
    PlayDecider() = default;

    /**
     * Decides which play to choose out of the valid plays. Selects the play that best fits the world and field state. If you want to
     * do meta logic, like making the play selection more aggressive or risky, you should do it in this function
     * @param world the current world state
     * @param field the current field state
     * @param validPlays a vector of plays which have been vetted by the PlayDecider and are deemed appropriate for this situaton
     * @return the play that best fits the world, and field.
     */
    Play* decideBestPlay(world_new::view::WorldDataView world, const world::Field& field, std::vector<Play*> const& validPlays);

   private:
    /**
     * local variable to keep track of which play is the best for the tick
     */
    Play* bestPlay = nullptr;
};
}  // namespace rtt::ai::analysis

#endif  // RTT_PLAYDECIDER_H
