//
// Created by Dawid Kulikowski on 08/08/2021.
//

#include "interface/InterfaceController.h"

#include "roboteam_utils/Print.h"

namespace rtt::Interface {

std::optional<proto::ModuleState> InterfaceController::getChanges() {
    if (!this->stateHandler->getState()) {
        return std::nullopt;
    }

    RTT_WARNING("AI changed interface value!");

    proto::ModuleState state;
    proto::Handshake handshake;

    handshake.set_module_name("_INTERFACE");
    handshake.mutable_declarations()->CopyFrom(this->declarations->toProto());
    handshake.mutable_values()->CopyFrom(this->settings->toProto());

    state.mutable_handshakes()->AddAllocated(&handshake);

    return state;
}

void InterfaceController::handleUpdate(proto::UiValues val) {
    this->settings->handleData(val);
//    this->declarations->handleData(state.handshakes(0).declarations());
}
}