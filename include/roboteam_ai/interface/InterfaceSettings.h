//
// Created by Dawid Kulikowski on 09/08/2021.
//

#ifndef RTT_INTERFACESETTINGS_H
#define RTT_INTERFACESETTINGS_H
#include "roboteam_proto/UiOptions.pb.h"
#include "InterfaceDeclarations.h"
#include "InterfaceValue.h"
#include "InterfaceStateHandler.h"

namespace rtt::Interface {
enum class InterfaceSettingsPrecedence { AI, IFACE };

class InterfaceSettings {
   private:
    std::map<std::string, InterfaceValue> values = {};

    mutable std::mutex mtx;

    std::weak_ptr<InterfaceStateHandler> stateHandler;

    void _unsafeSetSetting(const std::string name, const InterfaceValue newValue);

   public:
    InterfaceSettings(std::weak_ptr<InterfaceStateHandler> sts): stateHandler(sts) {};

    std::optional<InterfaceValue> getSetting(const std::string name) const;
    void setSetting(const std::string name, const InterfaceValue newValue);

    void handleData(const proto::UiValues&, InterfaceSettingsPrecedence = InterfaceSettingsPrecedence::AI);

    proto::UiValues toProto();
};
}  // namespace rbtt::Interface

#endif  // RTT_INTERFACESETTINGS_H
