#include "interface/InterfaceControllerServer.h"

namespace rtt::Interface {

bool InterfaceControllerServer::hasPriorityData() const noexcept { return true; }

void InterfaceControllerServer::handleData(const proto::UiValues &state) { this->vals->handleData(state, this->decls); }

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

}  // namespace rtt::Interface