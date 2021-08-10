//
// Created by Dawid Kulikowski on 08/08/2021.
//

#include "interface/Interface.h"
#include "roboteam_utils/Print.h"

void Interface::registerChange() {
    this->doesNeedUpdate.store(true);
}

std::optional<proto::ModuleState> Interface::getChanges() {
    bool expect = true;

    // It's not very important that this succeeds
    bool didExchange = this->doesNeedUpdate.compare_exchange_strong(expect, false);

    if (!didExchange && !expect) {
        return std::nullopt;
    }

    RTT_WARNING("AI changed interface value!");
//    TODO: Go through each component and compile new ModuleState

    return std::nullopt;
}