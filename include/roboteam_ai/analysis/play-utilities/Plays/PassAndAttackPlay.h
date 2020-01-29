//
// Created by jessevw on 29.01.20.
//

#ifndef RTT_PASSANDATTACKPLAY_H
#define RTT_PASSANDATTACKPLAY_H


//
// Created by jessevw on 14.01.20.
//

#ifndef RTT_PASSANDPLAYPLAY_H
#define RTT_PASSANDPLAYPLAY_H

#include <include/roboteam_ai/utilities/RobotDealer.h>
#include "analysis/play-utilities/Play.h"
#include "bt/BehaviorTree.hpp"
namespace rtt::ai::analysis {
    class PassAndAttackPlay : public Play {
    public:
        PassAndAttackPlay(std::string name);

        void executePlay(world::World* world, world::Field* field);

        /**
         * Decides if the play is valid to keep playing for the given world
         * @param world the current world
         * @param field the current field
         * @return true if the play is valid, false otherwise
         */
        virtual bool isValidPlayToKeep(rtt::ai::world::World* world, rtt::ai::world::Field* field);

        virtual bool isValidPlayToStart(world::World *world, world::Field *field);

        /**
         * @param world
         * @param field
         * @return the internal score of the play, based on the world and the field (and thus no meta strategy)
         */
        virtual uint8_t scorePlay(rtt::ai::world::World* world, rtt::ai::world::Field* field);

    private:
        /**
         * The behaviour trees this tactic consists of. Should be in a list, will be done later.
         */
        void makeTree();

    };
}



#endif //RTT_PASSANDPLAYPLAY_H


#endif //RTT_PASSANDATTACKPLAY_H
