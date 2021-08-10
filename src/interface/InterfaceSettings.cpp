//
// Created by Dawid Kulikowski on 10/08/2021.
//

#include "interface/InterfaceSettings.h"

const std::optional<InterfaceValue> InterfaceSettings::getSetting(const std::string name) const {
    std::scoped_lock(this->mtx);

    if (!this->values.contains(name)) {
        return std::nullopt;
    }

    return this->values.at(name);
}
void InterfaceSettings::setSetting(const std::string name, const InterfaceValue newValue) {
    std::scoped_lock(this->mtx);

    this->_unsafeSetSetting(name, newValue);
}

void InterfaceSettings::_unsafeSetSetting(const std::string name, const InterfaceValue newValue) {
    this->values.insert_or_assign(name, newValue);
}

void InterfaceSettings::handleData(const proto::UiValues& values, InterfaceDeclarations& decls, InterfaceSettingsPrecedence precedence) {
    std::scoped_lock(this->mtx);

    // TODO: Verify if value is in declarations, and whether it is an allowed value
    // Also, if precedence lvl == AI, we are AI and should verify that the value we are changing is modifiable by interface

    if (values.ui_values().size() <= 0) {
        RTT_WARNING("[Interface] Values are empty!")
        return;
    }

    this->values.clear();

    for(auto entry : values.ui_values()) {
        this->_unsafeSetSetting(entry.first, entry.second);
    }
}