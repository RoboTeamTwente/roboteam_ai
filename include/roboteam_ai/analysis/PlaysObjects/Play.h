//
// Created by jessevw on 06.12.19.
//

#ifndef RTT_PLAY_H
#define RTT_PLAY_H

#include <vector>
#include "include/roboteam_ai/analysis/PlaysObjects/Invariants/Invariant.h"
#include "bt/BehaviorTree.hpp"

namespace rtt::ai::analysis {
    /**
     * The play has a vector of invariants, when the invariants are false the play is abandoned.
     *
     * TODO: Should this object have a behaviourtree associated to it? I think yes
     */
    class Play {
    public:
        Play();

        Play(std::string name, std::vector<std::shared_ptr<Invariant>> invariants);

        void setInvariants(const std::vector<std::shared_ptr<Invariant>> &invariants);
        const std::vector<std::shared_ptr<Invariant>> &getInvariants() const;


        /**
         *
         * @return true if all the invariants of this strategy are true
         */
        bool isValidPlay(rtt::ai::world::World* world, rtt::ai::world::Field* field) ;

        std::string getName();
    protected:
        std::vector<std::shared_ptr<Invariant>> invariants;
        Invariant inv;
        std::shared_ptr<bt::BehaviorTree> tree;
        std::string name;

    };
}



#endif //RTT_PLAY_H
