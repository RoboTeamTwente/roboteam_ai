//
// Created by Dawid Kulikowski on 06/01/2022.
//

#ifndef RTT_INTERFACECONTROLLERSERVER_H
#define RTT_INTERFACECONTROLLERSERVER_H

#include <roboteam_interface_utils/InterfaceController.h>
#include <utilities/IOManager.h>
#include <utils/Channels.hpp>

namespace rtt::Interface {
    class InterfaceControllerServer: public InterfaceController<proto::ModuleState, proto::UiValues> {
        InterfaceControllerServer() : InterfaceController<proto::ModuleState, proto::UiValues>(rtt::net::utils::ChannelType::AI_TO_INTERFACE_CHANNEL, rtt::net::utils::ChannelType::INTERFACE_TO_AI_CHANNEL, 1, 0) {}

        bool hasPriorityData() const noexcept override;

        void handleData(const proto::UiValues &state) override;

        proto::ModuleState getDataForRemote(bool expired) const noexcept override;
    };
}

#endif  // RTT_INTERFACECONTROLLERSERVER_H
