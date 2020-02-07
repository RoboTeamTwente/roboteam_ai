//
// Created by jessevw on 19.11.19.
//

#ifndef RTT_BEINGPASSEDTOHELPER_H
#define RTT_BEINGPASSEDTOHELPER_H
#include "bt/BehaviorTree.hpp"

namespace bt {
class BeingPassedToHelper {
   public:
    BeingPassedToHelper();

    std::shared_ptr<bt::Node> createBeingPassedToChecker();
};
}  // namespace bt

#endif  // RTT_BEINGPASSEDTOHELPER_H
