//
// Created by Dawid Kulikowski on 06/01/2022.
//

#ifndef RTT_INTERFACECONTROLLERSERVER_H
#define RTT_INTERFACECONTROLLERSERVER_H

#include <roboteam_interface_utils/InterfaceController.h>
#include <utilities/IOManager.h>

namespace rtt::Interface {
    class InterfaceControllerServer: public InterfaceController<16970, 1, 0, proto::ModuleState, proto::UiValues> {
        bool hasPriorityData() const noexcept override {
            return true;
        }

        void handleData(const proto::UiValues &state) override {
            this->vals->handleData(state, decls);
        }

        proto::ModuleState getDataForRemote(bool expired) const noexcept override {
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
    };
}


#endif  // RTT_INTERFACECONTROLLERSERVER_H
