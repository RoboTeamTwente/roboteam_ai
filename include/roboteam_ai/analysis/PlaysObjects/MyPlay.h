//
// Created by jessevw on 06.12.19.
//

#ifndef RTT_MYPLAY_H
#define RTT_MYPLAY_H


#include "analysis/PlaysObjects/Play.h"

namespace rtt::ai::analysis {
    class MyPlay : public Play {
    public:
        MyPlay(std::vector<Invariant>& invariants);
        bool isValidPlay(rtt::ai::world::World * world, rtt::ai::world::Field * field);

    };

}


#endif //RTT_MYPLAY_H
