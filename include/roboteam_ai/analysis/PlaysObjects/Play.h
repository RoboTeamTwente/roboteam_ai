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
     * TODO: Should this object have a behaviourtree associated to it? I think yes
     */
    class Play {
    public:
        Play();

        Play(std::string name, std::vector<std::function<bool(world::World* , world::Field*)>> invariants);

        /**
         *
         * @return true if all the invariants of this strategy are true
         */
        virtual bool isValidPlay(rtt::ai::world::World* world, rtt::ai::world::Field* field);

        virtual int scorePlay(rtt::ai::world::World* world, rtt::ai::world::Field* field);

        std::string getName();

        virtual std::shared_ptr<bt::BehaviorTree> getTree();

    protected:
        std::vector<std::function<bool(world::World* , world::Field*)>> invariants;
        std::shared_ptr<bt::BehaviorTree> tree;
        std::string name;

        //virtual void executePlay();
    };
}



#endif //RTT_PLAY_H
