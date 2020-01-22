//
// Created by jessevw on 21.01.20.
//

#ifndef RTT_DUMMYPLAY_H
#define RTT_DUMMYPLAY_H

#include <include/roboteam_ai/utilities/RobotDealer.h>
#include "analysis/PlaysObjects/Play.h"

namespace rtt::ai::analysis {
    class DummyPlay : public Play {
    public:
        //std::vector<std::pair<std::string, rtt::ai::robotDealer::RobotType>> robots;
        DummyPlay(std::string name);
        void executePlay(world::World* world, world::Field* field);
        std::shared_ptr<bt::BehaviorTree> getTreeForWorld();
        void moveToNextTactic();
        virtual bool isValidPlay(rtt::ai::world::World* world, rtt::ai::world::Field* field);

        virtual int scorePlay(rtt::ai::world::World* world, rtt::ai::world::Field* field);

    private:
        std::shared_ptr<bt::BehaviorTree> tree1;
        // std::shared_ptr<bt::BehaviorTree> tree2;

    };
}


#endif //RTT_DUMMYPLAY_H
