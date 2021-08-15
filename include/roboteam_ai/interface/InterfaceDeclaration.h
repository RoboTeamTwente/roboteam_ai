//
// Created by Dawid Kulikowski on 10/08/2021.
//

#ifndef RTT_INTERFACEDECLARATION_H
#define RTT_INTERFACEDECLARATION_H
#include <variant>
#include "InterfaceValue.h"
#include <roboteam_proto/UiOptions.pb.h>
#include <nlohmann/json.hpp>

namespace rbtt::Interface {
struct InterfaceSlider {
    std::string text;
    float min;
    float max;
    float interval;

    InterfaceSlider() = default;
    InterfaceSlider(const proto::Slider&);
    InterfaceSlider(const std::string, const float, const float, const float);

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(InterfaceSlider, text, min, max, interval);
};

struct InterfaceCheckbox {
    std::string text;

    InterfaceCheckbox() = default;
    InterfaceCheckbox(const proto::Checkbox&);
    InterfaceCheckbox(const std::string);

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(InterfaceCheckbox, text);
};

struct InterfaceDropdown {
    std::string text;
    std::vector<std::string> options;

    InterfaceDropdown() = default;
    InterfaceDropdown(const proto::Dropdown&);
    InterfaceDropdown(const std::string, const std::vector<std::string>);

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(InterfaceDropdown, text, options);

};

struct InterfaceRadio {
    std::vector<std::string> options;

    InterfaceRadio() = default;
    InterfaceRadio(const proto::RadioButton&);
    InterfaceRadio(const std::vector<std::string>);

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(InterfaceRadio, options);
};

struct InterfaceText {
    std::string text;

    InterfaceText() = default;
    InterfaceText(const proto::TextField&);
    InterfaceText(const std::string);

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(InterfaceText, text);
};

typedef std::variant<std::monostate, rbtt::Interface::InterfaceSlider, InterfaceCheckbox, InterfaceDropdown, InterfaceRadio, InterfaceText> InterfaceOptions;

struct InterfaceDeclaration {
   public:
    std::string path;
    std::string description;

    bool isMutable;

    InterfaceValue defaultValue;

    InterfaceOptions options;

    proto::UiOptionDeclaration toProtoMessage();


    InterfaceDeclaration() = default;
    InterfaceDeclaration(const proto::UiOptionDeclaration&);
    InterfaceDeclaration(const std::string, const std::string, const bool, const InterfaceValue, const InterfaceOptions);
};

}  // namespace rbtt::Interface

#endif  // RTT_INTERFACEDECLARATION_H
