//
// Created by jessevw on 19.11.19.
//

#ifndef RTT_BEINGPASSEDTOHELPER_H
#define RTT_BEINGPASSEDTOHELPER_H
#include "bt/BehaviorTree.hpp"

/**
 * Wonder if this should be a class in itself as it is now, or part of a class and as a function.
 */

namespace bt {
    class BeingPassedToHelper {
    public:
        BeingPassedToHelper();

        std::shared_ptr<bt::Node> createBeingPassedToChecker();

    };
}

#endif //RTT_BEINGPASSEDTOHELPER_H
