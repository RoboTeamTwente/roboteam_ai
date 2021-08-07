//
// Created by Dawid Kulikowski on 07/08/2021.
//

#ifndef RTT_INTERFACESETTING_H
#define RTT_INTERFACESETTING_H

#include <optional>
#include <roboteam_proto/UiOptions.pb.h>
#include <roboteam_proto/State.pb.h>

class InterfaceSettingStore {
   public:
    [[nodiscard]] virtual std::optional<proto::UiValue> getSetting(const std::string& name) const = 0;
};

class InterfaceSetting: public InterfaceSettingStore {
   private:
    proto::UiValues values;
   public:
    void handleNewData(const proto::ModuleState newState);
};

#endif  // RTT_INTERFACESETTING_H
