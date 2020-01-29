//
// Created by jessevw on 06.12.19.
//

#ifndef RTT_PLAY_H
#define RTT_PLAY_H

#include <vector>
#include "bt/BehaviorTree.hpp"
#include "functional"

namespace rtt::ai::analysis {
/**
 * The play has a vector of invariants, when the invariants are false the play is abandoned.
 */
    class Play {
    public:
        Play() = default;

        Play(std::string name);

        /**
         * Check if we can keep playing this play
         * @param world the current world
         * @param field the current field
         * @return true if invariants of play are true, false otherwise
         */
        virtual bool isValidPlayToKeep(world::World* world, world::Field* field) = 0;

        /**
         * Check if we can start this play.
         * @param world the current world
         * @param field the current field
         * @return true if preconditions are true, false otherwise
         */
        virtual bool isValidPlayToStart(world::World *world, world::Field *field) = 0;

        // TODO: Move this to the derived class
        /**
         * Returns a score based on how fitting this play is given a world and field state (currently hardcoded, should be moved to derived classes)
         * @param world the current world
         * @param field the current field
         * @return a score between 0 and 10, 10 being the best
         */
        virtual uint8_t scorePlay(world::World *world, world::Field *field) = 0;

        std::string_view getName();

        const std::shared_ptr<bt::BehaviorTree> &getTree() const;

    protected:
        /**
         * Internal tree of the play, where the execution of the play is done
         */
        std::shared_ptr<bt::BehaviorTree> tree;
        std::string name;
    };
}  // namespace rtt::ai::analysis

#endif  // RTT_PLAY_H
