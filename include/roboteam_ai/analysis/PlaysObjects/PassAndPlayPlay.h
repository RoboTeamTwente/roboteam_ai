//
// Created by jessevw on 14.01.20.
//

#ifndef RTT_PASSANDPLAYPLAY_H
#define RTT_PASSANDPLAYPLAY_H

#include "analysis/PlaysObjects/Play.h"
namespace rtt::ai::analysis {
    class PassAndPlayPlay : Play {
    public:
        PassAndPlayPlay();
        bt::Node::Status executePlay(world::World* world, world::Field* field);

    };
}



#endif //RTT_PASSANDPLAYPLAY_H
