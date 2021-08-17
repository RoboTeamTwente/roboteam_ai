//
// Created by Dawid Kulikowski on 10/08/2021.
//

#ifndef RTT_INTERFACEDECLARATION_H
#define RTT_INTERFACEDECLARATION_H
#include <variant>
#include "InterfaceValue.h"
#include <roboteam_proto/UiOptions.pb.h>
#include <nlohmann/json.hpp>

namespace rtt::Interface {
struct InterfaceSlider {
    std::string text;
    float min;
    float max;
    float interval;

    InterfaceSlider() = default;
    InterfaceSlider(const proto::Slider&);
    InterfaceSlider(const std::string, const float, const float, const float);

    proto::Slider toProto() const {
        proto::Slider slider;

        slider.set_text(this->text);
        slider.set_min(this->min);
        slider.set_max(this->max);
        slider.set_interval(this->interval);

        return slider;
    }

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(InterfaceSlider, text, min, max, interval);
};

struct InterfaceCheckbox {
    std::string text;

    InterfaceCheckbox() = default;
    InterfaceCheckbox(const proto::Checkbox&);
    InterfaceCheckbox(const std::string);

    proto::Checkbox toProto() const {
        proto::Checkbox checkbox;

        checkbox.set_text(this->text);

        return checkbox;
    }

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(InterfaceCheckbox, text);
};

struct InterfaceDropdown {
    std::string text;
    std::vector<std::string> options;

    InterfaceDropdown() = default;
    InterfaceDropdown(const proto::Dropdown&);
    InterfaceDropdown(const std::string, const std::vector<std::string>);

    proto::Dropdown toProto() const {
        proto::Dropdown dropdown;

        dropdown.set_text(this->text);
        dropdown.mutable_options()->Add(options.begin(), options.end());

        return dropdown;
    }

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(InterfaceDropdown, text, options);

};

struct InterfaceRadio {
    std::vector<std::string> options;

    InterfaceRadio() = default;
    InterfaceRadio(const proto::RadioButton&);
    InterfaceRadio(const std::vector<std::string>);

    proto::RadioButton toProto() const {
        proto::RadioButton radio;

        radio.mutable_options()->Add(options.begin(), options.end());
        return radio;
    }

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(InterfaceRadio, options);
};

struct InterfaceText {
    std::string text;

    InterfaceText() = default;
    InterfaceText(const proto::TextField&);
    InterfaceText(const std::string);

    proto::TextField toProto() const {
        proto:: TextField txtField;

        txtField.set_text(text);

        return txtField;
    }

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(InterfaceText, text);
};

typedef std::variant<std::monostate, InterfaceSlider, InterfaceCheckbox, InterfaceDropdown, InterfaceRadio, InterfaceText> InterfaceOptions;

struct InterfaceDeclaration {
   public:
    std::string path;
    std::string description;

    bool isMutable;

    InterfaceValue defaultValue;

    InterfaceOptions options;

    proto::UiOptionDeclaration toProto() const;


    InterfaceDeclaration() = default;
    InterfaceDeclaration(const proto::UiOptionDeclaration&);
    InterfaceDeclaration(const std::string, const std::string, const bool, const InterfaceValue, const InterfaceOptions);
};

}  // namespace rbtt::Interface

#endif  // RTT_INTERFACEDECLARATION_H
