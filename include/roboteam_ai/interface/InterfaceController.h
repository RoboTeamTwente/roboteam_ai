//
// Created by Dawid Kulikowski on 08/08/2021.
//

#ifndef RTT_INTERFACECONTROLLER_H
#define RTT_INTERFACECONTROLLER_H

#include "InterfaceSettings.h"

#include <roboteam_proto/State.pb.h>
#include <atomic>
#include <optional>

namespace rbtt::Interface {
class InterfaceController {
   private:
    std::shared_ptr<InterfaceSettings> settings;
    std::shared_ptr<InterfaceDeclarations> declarations;

    std::atomic<bool> doesNeedUpdate = false;

   public:
    InterfaceController(): settings(std::make_shared<InterfaceSettings>()), declarations(std::make_shared<InterfaceDeclarations>()) {}
    InterfaceController(InterfaceSettings npsettings, InterfaceDeclarations decls): settings(std::make_shared<InterfaceSettings>(npsettings)), declarations(std::make_shared<InterfaceDeclarations>(decls)) {}

    void handleUpdate(proto::ModuleState);

    void registerChange();
    std::optional<proto::ModuleState> getChanges();

    std::weak_ptr<InterfaceSettings> getSettings() {return settings;}
};
}

#endif  // RTT_INTERFACECONTROLLER_H
