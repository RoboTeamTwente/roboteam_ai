//
// Created by jessevw on 06.12.19.
//

#include "include/roboteam_ai/analysis/PlaysObjects/MyPlay.h"
namespace rtt::ai::analysis{
    /**
     * derived class of Play. isValidPlay does not need to be overridden here. This class should have a behaviour tree associated with it
     * @param invariants
     */
    MyPlay::MyPlay(std::string name, std::vector<std::shared_ptr<Invariant>> invariants) : Play(name, invariants) {
        this->tree = nullptr;
    }

}