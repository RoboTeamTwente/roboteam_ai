//
// Created by jessevw on 31.10.19.
//

#include "treeinterp/StrategyInterface.h"

namespace bt {
    StrategyInterface::StrategyInterface() {
        tree = std::make_shared<BehaviorTree>();
    }
    StrategyInterface::StrategyInterface(std::list<DefaultTactic> tacticlist) : StrategyInterface(){
        this->setTactics(tacticlist);
    }
    void StrategyInterface::createStrategy() {

    }
    void StrategyInterface::createTactics() {

    }
    std::list<DefaultTactic> StrategyInterface::getTactic() {
        return tactics;
    }
}