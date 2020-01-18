//
// Created by jessevw on 14.01.20.
//

#ifndef RTT_PASSANDPLAYPLAY_H
#define RTT_PASSANDPLAYPLAY_H

#include <include/roboteam_ai/utilities/RobotDealer.h>
#include "analysis/PlaysObjects/Play.h"
namespace rtt::ai::analysis {
    class PassAndPlayPlay : public Play {
    public:
        //std::vector<std::pair<std::string, rtt::ai::robotDealer::RobotType>> robots;
        PassAndPlayPlay();
        bt::Node::Status executePlay(world::World* world, world::Field* field);
    };
}



#endif //RTT_PASSANDPLAYPLAY_H
