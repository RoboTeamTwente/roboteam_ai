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
        PassAndPlayPlay(std::string name);
        void executePlay(world::World* world, world::Field* field);

        /**
         * function that checks if we need to move to the next tree in the play, and does so if necessary.
         * @return if we need to use the next tree, it returns the next tree in the play, else, it returns the current tree in the play
         */
        std::shared_ptr<bt::BehaviorTree> getTreeForWorld();

        /**
         * Checks if we need to move to the next tree in the play.
         */
        void moveToNextTactic();

         /**
          * Decides if the play is valid to keep playing for the given world
          * @param world the current world
          * @param field the current field
          * @return true if the play is valid, false otherwise
          */
        virtual bool isValidPlay(rtt::ai::world::World* world, rtt::ai::world::Field* field);

        /**
         * @param world
         * @param field
         * @return the internal score of the play, based on the world and the field (and thus no meta strategy)
         */
        virtual int scorePlay(rtt::ai::world::World* world, rtt::ai::world::Field* field);

    private:
        /**
         * The behaviour trees this tactic consists of. Should be in a list, will be done later.
         */
         void makeTree1();
         void makeTree2();
        std::shared_ptr<bt::BehaviorTree> tree1;
        std::shared_ptr<bt::BehaviorTree> tree2;

    };
}



#endif //RTT_PASSANDPLAYPLAY_H
