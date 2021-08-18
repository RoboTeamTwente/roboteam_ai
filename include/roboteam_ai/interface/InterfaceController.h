//
// Created by Dawid Kulikowski on 08/08/2021.
//

#ifndef RTT_INTERFACECONTROLLER_H
#define RTT_INTERFACECONTROLLER_H

#include "roboteam_interface/InterfaceSettings.h"
#include "roboteam_interface/InterfaceStateHandler.h"
#include "roboteam_interface/InterfaceDeclarations.h"

#include <roboteam_proto/State.pb.h>
#include <atomic>
#include <optional>


namespace rtt::Interface {
class InterfaceController {
   private:
    std::shared_ptr<InterfaceSettings> settings;
    std::shared_ptr<InterfaceDeclarations> declarations;
//
    std::shared_ptr<InterfaceStateHandler> stateHandler;

   public:
    InterfaceController(): declarations(std::make_shared<InterfaceDeclarations>(stateHandler)), settings(std::make_shared<InterfaceSettings>(stateHandler)) {}

    void handleUpdate(proto::UiValues);


    std::optional<proto::ModuleState> getChanges();

    std::weak_ptr<InterfaceSettings> getSettings() {return settings;}
    std::weak_ptr<InterfaceDeclarations> getDeclarations() {return declarations;}
};
}

#endif  // RTT_INTERFACECONTROLLER_H
