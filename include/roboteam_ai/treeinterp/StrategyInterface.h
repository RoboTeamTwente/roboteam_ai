//
// Created by jessevw on 31.10.19.
//

#ifndef RTT_STRATEGYINTERFACE_H
#define RTT_STRATEGYINTERFACE_H
#include <include/roboteam_ai/bt/tactics/DefaultTactic.h>
#include "bt/BehaviorTree.hpp"
#include "bt/Role.h"
#include "TreeProtoType.h"

namespace bt {

    class StrategyInterface {
    private:
        /** A strategy can have multile tactics. Some tactics need
        to take precedence over others in the robotdealer, so
        a list is used, with the most important tactic at index 0 */
        std::list<DefaultTactic> tactics;
        std::shared_ptr<BehaviorTree> tree;
    public:
        /**
         * Creates a tactic TODO add proper documentation to this
         */
        virtual void createStrategy();
        virtual void createTactics();

        StrategyInterface();
        StrategyInterface(std::list<DefaultTactic> tactics);

        // getters and setters for the tactics in the strategy
        void setTactics(std::list<DefaultTactic>);
        std::list<DefaultTactic> getTactic();


    };
}

#endif //RTT_STRATEGYINTERFACE_H
