//
// Created by Dawid Kulikowski on 06/01/2022.
//

#ifndef RTT_INTERFACECONTROLLERSERVER_H
#define RTT_INTERFACECONTROLLERSERVER_H

#include <roboteam_interface_utils/InterfaceController.h>
#include <utilities/IOManager.h>

namespace rtt::Interface {
    class InterfaceControllerServer: public InterfaceController<16970, 1, 0, proto::ModuleState, proto::UiValues> {
        bool hasPriorityData() const noexcept override;

        void handleData(const proto::UiValues &state) override;

        proto::ModuleState getDataForRemote(bool expired) const noexcept override;
    };
}

#endif  // RTT_INTERFACECONTROLLERSERVER_H
