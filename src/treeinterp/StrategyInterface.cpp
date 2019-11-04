//
// Created by jessevw on 31.10.19.
//

#include "treeinterp/StrategyInterface.h"

namespace bt {
    /**
     * This interface is used to construct behaviour trees with. The interface contains several methods necessary to create a strategy.
     * These methods are enforced in derived classes through the virtual contract.
     */
    StrategyInterface::StrategyInterface() {
        tree = std::make_shared<BehaviorTree>();
    }

    StrategyInterface::StrategyInterface(std::list<DefaultTactic> tacticlist) : StrategyInterface(){
        this->setTactic(tacticlist);
    }
    /**
     * Builds the BehaviourTree for this strategy (overwrite in subclass)
     */
    void StrategyInterface::createStrategy() {

    }

    /**
     * This function creates the tactics for this strategy. Derived classes should implement this function so that they
     * get the behaviour of the tactics that they want.
     */
    void StrategyInterface::createTactics(){

    }

    /**
     * @pure
     * @return the tactics variable for this strategyinterface. The tactics variable is a list of
     */
    std::list<DefaultTactic> StrategyInterface::getTactic() {
        return tactics;
    }
    void StrategyInterface::setTactic(std::list<DefaultTactic> tacticslist) {
        tactics = tacticslist;
    }
    /**
     * Gets the BehaviorTree for this strategy. This is the tree that should be ticked if this strategy is desired
     * to be ran. In order to create this tree, the method createStrategy() must first be called to create the tree.
     * @return the BehaviorTree needed to run this strategy
     */
    std::shared_ptr<bt::BehaviorTree> StrategyInterface::getStrategyTree() {
        return tree;
    }
}