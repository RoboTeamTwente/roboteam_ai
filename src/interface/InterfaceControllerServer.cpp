#include "interface/InterfaceControllerServer.h"

namespace rtt::Interface {


void InterfaceControllerServer::handleData(const proto::UiValues &state) { this->vals->handleData(state, this->decls); }

proto::ModuleState InterfaceControllerServer::getDataForRemote() const noexcept {
    proto::ModuleState mod;
    proto::Handshake hand;
    hand.set_module_name("IFACE");

    auto val = this->vals->toProto();
    hand.mutable_values()->Swap(&val);

    mod.mutable_handshakes()->Add(std::move(hand));

    return mod;
}

void InterfaceControllerServer::loop() {
    while (this->shouldRun) {
        this->update_marker.acquire();
        loop_iter();
    }
}

void InterfaceControllerServer::trigger_update() {
    this->update_marker.release();
}

}  // namespace rtt::Interface