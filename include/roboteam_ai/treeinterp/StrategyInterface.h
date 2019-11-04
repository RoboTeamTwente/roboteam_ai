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
        /** A strategy can have multiple tactics. Some tactics need
        to take precedence over others in the RobotDealer, so
        a list is used, with the most important tactic at index 0 */
        std::list<DefaultTactic> tactics;
        std::shared_ptr<BehaviorTree> tree;
    public:
        /**
         * Creates a tactic TODO add proper documentation to this
         */
        // Virtual methods
        virtual void createStrategy();
        virtual void createTactics();

        // constructors
        StrategyInterface();
        StrategyInterface(std::list<DefaultTactic> tactics);


        // getters and setters
        void setTactic(std::list<DefaultTactic>);
        std::list<DefaultTactic> getTactic();

        std::shared_ptr<bt::BehaviorTree> getStrategyTree();


    };
}

#endif //RTT_STRATEGYINTERFACE_H
