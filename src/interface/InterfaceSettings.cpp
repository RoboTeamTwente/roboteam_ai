//
// Created by Dawid Kulikowski on 10/08/2021.
//

#include "interface/InterfaceSettings.h"
#include "roboteam_utils/Print.h"

namespace rtt::Interface {

std::optional<InterfaceValue> InterfaceSettings::getSetting(const std::string name) const {
    std::scoped_lock lck(this->mtx);

    if (!this->values.contains(name)) {
        return std::nullopt;
    }

    return this->values.at(name);
}
void InterfaceSettings::setSetting(const std::string name, const InterfaceValue newValue) {
    std::scoped_lock lck(this->mtx);

    this->_unsafeSetSetting(name, newValue);

    if (auto iptr = this->stateHandler.lock()) {
        iptr->stateDidChange();
    }
}

void InterfaceSettings::_unsafeSetSetting(const std::string name, const InterfaceValue newValue) { this->values.insert_or_assign(name, newValue); }

void InterfaceSettings::handleData(const proto::UiValues& values, InterfaceSettingsPrecedence precedence) {
    std::scoped_lock lck(this->mtx);

    // TODO: Verify if value is in declarations, and whether it is an allowed value
    // Also, if precedence lvl == AI, we are AI and should verify that the value we are changing is modifiable by interface

    values.PrintDebugString();
    if (values.ui_values().empty()) {
        RTT_WARNING("[Interface] Values are empty!");
        return;
    }

    this->values.clear();

    for (const auto& entry : values.ui_values()) {
        this->_unsafeSetSetting(entry.first, entry.second);
    }
}
proto::UiValues InterfaceSettings::toProto() {
    std::scoped_lock lck(this->mtx);
    proto::UiValues vals;

    for (const auto& [key, value] : this->values) {
        vals.mutable_ui_values()->insert({key, value.toProto()});
    }

    return vals;
}

}