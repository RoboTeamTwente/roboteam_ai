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
#include <fstream>
#include <nlohmann/json.hpp>


namespace rtt {
class InterfaceController {
   private:
    std::shared_ptr<Interface::InterfaceSettings> settings;
    std::shared_ptr<Interface::InterfaceDeclarations> declarations;
//
std::shared_ptr<Interface::InterfaceStateHandler> stateHandler;

   public:
    InterfaceController(const std::string);
    InterfaceController(): declarations(std::make_shared<Interface::InterfaceDeclarations>(stateHandler)), settings(std::make_shared<Interface::InterfaceSettings>(stateHandler)) {}

    void handleUpdate(proto::UiValues);


    std::optional<proto::ModuleState> getChanges();

    std::weak_ptr<Interface::InterfaceSettings> getSettings() {return settings;}
    std::weak_ptr<Interface::InterfaceDeclarations> getDeclarations() {return declarations;}
};
}

#endif  // RTT_INTERFACECONTROLLER_H
