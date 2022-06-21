//
// Created by Dawid Kulikowski on 06/01/2022.
//

#ifndef RTT_INTERFACECONTROLLERSERVER_H
#define RTT_INTERFACECONTROLLERSERVER_H

#include <roboteam_interface_utils/InterfaceController.h>
#include <utilities/IOManager.h>
#include <semaphore>

#include <utils/Channels.hpp>

namespace rtt::Interface {
class InterfaceControllerServer : public InterfaceController<proto::ModuleState, proto::UiValues> {
   public:
    InterfaceControllerServer()
        : InterfaceController<proto::ModuleState, proto::UiValues>(rtt::net::utils::ChannelType::AI_TO_INTERFACE_CHANNEL, rtt::net::utils::ChannelType::INTERFACE_TO_AI_CHANNEL) {}

    void loop() override;

    void handleData(const proto::UiValues &state) override;

    proto::ModuleState getDataForRemote() const noexcept override;
    void trigger_update();
    std::string getSelectedPlay();

   private:
    std::binary_semaphore update_marker{1};
    std::string selectedPlay;

    void setSelectedPlay(const std::string& newPlay);
};
}  // namespace rtt::Interface

#endif  // RTT_INTERFACECONTROLLERSERVER_H
