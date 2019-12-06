//
// Created by jessevw on 06.12.19.
//

#ifndef RTT_PLAY_H
#define RTT_PLAY_H

#include <vector>
#include "Invariant.h"

namespace rtt::ai::analysis {
    /**
     * The play has a vector of invariants, when the invariants are false the play is abandoned.
     *
     * TODO: Should this object have a behaviourtree associated to it? I think yes
     */
    class Play {
    public:
        Play(std::vector<Invariant> invariants, Invariant inv);
        void setInvariants(const std::vector<Invariant> &invariants);
        const std::vector<Invariant> &getInvariants() const;

    private:
        std::vector<Invariant> invariants;
        Invariant inv;

    };
}



#endif //RTT_PLAY_H
