#include "interface/InterfaceControllerServer.h"
#include <utilities/Settings.h>
#include <roboteam_utils/Print.h>

namespace rtt::Interface {

bool InterfaceControllerServer::hasPriorityData() const noexcept {
    return true;
}

void InterfaceControllerServer::handleData(const proto::UiValues &state) {
    this->vals->handleData(state, this->decls);

    for (const auto& val : state.ui_values()) {

        // Handle settings from the interface
        if (val.first == "IS_YELLOW") {
            if (val.second.has_bool_value()) {
                SETTINGS.setYellow(val.second.bool_value());
            }
        } else if (val.first == "IS_RIGHT") {
            if (val.second.has_bool_value()) {
                SETTINGS.setLeft(! val.second.bool_value());
            }
        } else if (val.first == "IGNORE_INVARIANTS") {
            if (val.second.has_bool_value()) {
                SETTINGS.setIgnoreInvariants(val.second.bool_value());
            }
        } else {
            std::cout << "Unhandled setting: '" << val.first << "'" << std::endl;
        }
    }
}

proto::ModuleState InterfaceControllerServer::getDataForRemote(bool expired) const noexcept {
    auto state = rtt::ai::io::io.getState();

    proto::ModuleState mod;
    mod.mutable_system_state()->Swap(&state);

    if (expired) {
        proto::Handshake hand;
        hand.set_module_name("IFACE");

        auto val = this->vals->toProto();
        hand.mutable_values()->Swap(&val);

        mod.mutable_handshakes()->Add(std::move(hand));
    }

    return mod;
}

} // namespace rtt::Interface