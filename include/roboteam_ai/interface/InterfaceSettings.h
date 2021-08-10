//
// Created by Dawid Kulikowski on 09/08/2021.
//

#ifndef RTT_INTERFACESETTINGS_H
#define RTT_INTERFACESETTINGS_H
#include "roboteam_proto/UiOptions.pb.h"
#include "InterfaceDeclarations.h"

enum class InterfaceSettingsPrecedence {
    AI, IFACE
};

class InterfaceSettings {
   private:
    std::map<std::string, InterfaceValue> values = {};
    std::weak_ptr<Interface> iface;

    mutable std::mutex mtx;


    void _unsafeSetSetting(const std::string name, const InterfaceValue newValue);
   public:
    InterfaceSettings(std::weak_ptr<Interface> interface): iface(interface) { };

    const std::optional<InterfaceValue> getSetting(const std::string name) const;
    void setSetting(const std::string name, const InterfaceValue newValue);


    void handleData(const proto::UiValues&, InterfaceDeclarations&, InterfaceSettingsPrecedence = InterfaceSettingsPrecedence::AI);
};

#endif  // RTT_INTERFACESETTINGS_H
