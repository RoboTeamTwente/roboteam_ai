//
// Created by Dawid Kulikowski on 10/08/2021.
//

#ifndef RTT_INTERFACEDECLARATION_H
#define RTT_INTERFACEDECLARATION_H
#include <variant>
#include "InterfaceValue.h"
#include <roboteam_proto/UiOptions.pb.h>

struct InterfaceSlider {
    std::string text;
    float min;
    float max;
    float interval;

    InterfaceSlider(const proto::Slider&);
    InterfaceSlider(const std::string, const float, const float, const float);
};

struct InterfaceCheckbox {
    std::string text;

    InterfaceCheckbox(const proto::Checkbox&);
    InterfaceCheckbox(const std::string);
};

struct InterfaceDropdown {
    std::string text;
    std::vector<std::string> options;

    InterfaceDropdown(const proto::Dropdown&);
    InterfaceDropdown(const std::string, const std::vector<std::string>);

};

struct InterfaceRadio {
    std::vector<std::string> options;
    InterfaceRadio(const proto::RadioButton&);
    InterfaceRadio(const std::vector<std::string>);
};

struct InterfaceText {
    std::string text;

    InterfaceText(const proto::TextField&);
    InterfaceText(const std::string);
};

typedef std::variant<std::monostate, InterfaceSlider, InterfaceCheckbox, InterfaceDropdown, InterfaceRadio, InterfaceText> InterfaceOptions;

struct InterfaceDeclaration {
    std::string path;
    std::string description;

    bool isMutable;

    InterfaceValue defaultValue;

    InterfaceOptions options;

    InterfaceDeclaration(const proto::UiOptionDeclaration&);
    InterfaceDeclaration(const std::string, const std::string, const bool, const InterfaceValue, const InterfaceOptions);
};

#endif  // RTT_INTERFACEDECLARATION_H
