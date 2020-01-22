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
     *
     */
    class Play {
    public:
        Play(std::string name);
        /**
         *
         * @return true if all the invariants of this strategy are true
         */
        virtual bool isValidPlay(rtt::ai::world::World* world, rtt::ai::world::Field* field) = 0;

        /**
         * returns a score for how good the play is for the current world and field states
         * @param world
         * @param field
         * @return
         */
        virtual int scorePlay(rtt::ai::world::World* world, rtt::ai::world::Field* field) = 0;

        virtual std::shared_ptr<bt::BehaviorTree> getTreeForWorld() = 0;

        std::string getName();

        virtual std::shared_ptr<bt::BehaviorTree> getTree();

    protected:
        std::shared_ptr<bt::BehaviorTree> tree;
        std::string name;

        //virtual void executePlay();
    };
}



#endif //RTT_PLAY_H
