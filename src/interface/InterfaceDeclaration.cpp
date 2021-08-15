//
// Created by Dawid Kulikowski on 10/08/2021.
//

#include "interface/InterfaceDeclaration.h"
#include <roboteam_utils/Print.h>
#include <exception>

namespace rbtt::Interface {

InterfaceDeclaration::InterfaceDeclaration(const proto::UiOptionDeclaration& decl) : defaultValue(InterfaceValue(decl.default_())) {
    this->path = decl.path();
    this->description = decl.description();
    this->isMutable = decl.is_mutable();

    switch (decl.ui_elements_case()) {
        case proto::UiOptionDeclaration::UiElementsCase::kCheckbox:
            this->options = decl.checkbox();
            break;
        case proto::UiOptionDeclaration::UiElementsCase::kDropdown:
            this->options = decl.dropdown();
            break;
        case proto::UiOptionDeclaration::UiElementsCase::kRadiobutton:
            this->options = decl.radiobutton();
            break;
        case proto::UiOptionDeclaration::UiElementsCase::kSlider:
            this->options = decl.slider();
            break;
        case proto::UiOptionDeclaration::UiElementsCase::kTextfield:
            this->options = decl.textfield();
            break;
        case proto::UiOptionDeclaration::UiElementsCase::UI_ELEMENTS_NOT_SET:
        default:
            throw std::logic_error{"Unknown variant type when initialising InterfaceDeclaration!"};
    }
}

void to_json(nlohmann::json& j, const InterfaceDeclaration& declaration) {
    nlohmann::json tmpJson =
        nlohmann::json{{"path", declaration.path}, {"description", declaration.description}, {"isMutable", declaration.isMutable}, {"defaultValue", declaration.defaultValue}};

    if (const auto* slider = std::get_if<rbtt::Interface::InterfaceSlider>(&declaration.options)) {
        tmpJson["slider"] = slider;
    } else if (const auto* checkbox = std::get_if<InterfaceCheckbox>(&declaration.options)) {
        tmpJson["checkbox"] = checkbox;
    } else if (const auto* dropdown = std::get_if<InterfaceDropdown>(&declaration.options)) {
        tmpJson["dropdown"] = dropdown;
    } else if (const auto* radio = std::get_if<InterfaceRadio>(&declaration.options)) {
        tmpJson["radio"] = radio;
    } else if (const auto* text = std::get_if<InterfaceText>(&declaration.options)) {
        tmpJson["text"] = text;
    } else {
        throw std::logic_error{"Variant was in an invalid state when serialising InterfaceDeclaration to JSON!"};
    }

    j = tmpJson;
}

void from_json(const nlohmann::json& json, InterfaceDeclaration& declaration) {
    json.at("path").get_to(declaration.path);
    json.at("description").get_to(declaration.description);

    json.at("isMutable").get_to(declaration.isMutable);
    json.at("defaultValue").get_to(declaration.defaultValue);

    //    TODO: Replace keys with constants
    if (json.contains("slider")) {
        declaration.options = json.at("slider").get<rbtt::Interface::InterfaceSlider>();
    } else if (json.contains("checkbox")) {
        declaration.options = json.at("checkbox").get<InterfaceCheckbox>();
    } else if (json.contains("dropdown")) {
        declaration.options = json.at("dropdown").get<InterfaceDropdown>();
    } else if (json.contains("radio")) {
        declaration.options = json.at("radio").get<InterfaceRadio>();
    } else if (json.contains("text")) {
        declaration.options = json.at("text").get<InterfaceText>();
    } else {
        throw std::domain_error{"Unknown type encountered when deserializing InterfaceDeclaration from JSON!"};
    }
}

InterfaceDeclaration::InterfaceDeclaration(const std::string path, const std::string description, const bool isMutable, const InterfaceValue defaultValue,
                                           const InterfaceOptions options)
    : defaultValue(defaultValue) {
    this->path = path;
    this->description = description;
    this->isMutable = isMutable;
    this->options = options;
}

InterfaceText::InterfaceText(const proto::TextField& protoTextField) { this->text = protoTextField.text(); }

InterfaceText::InterfaceText(const std::string text) { this->text = text; }

InterfaceRadio::InterfaceRadio(const proto::RadioButton& protoRadio) {
    // protobuf owns this message, we should copy it
    std::copy(protoRadio.options().begin(), protoRadio.options().end(), std::back_inserter(this->options));
}

InterfaceRadio::InterfaceRadio(const std::vector<std::string> options) { this->options = options; }

InterfaceDropdown::InterfaceDropdown(const proto::Dropdown& protoDropdown) {
    this->text = protoDropdown.text();

    std::copy(protoDropdown.options().begin(), protoDropdown.options().end(), std::back_inserter(this->options));
}

InterfaceDropdown::InterfaceDropdown(const std::string text, const std::vector<std::string> options) {
    this->text = text;
    this->options = options;
}

InterfaceCheckbox::InterfaceCheckbox(const proto::Checkbox& protoCheckbox) { this->text = protoCheckbox.text(); }

InterfaceCheckbox::InterfaceCheckbox(const std::string text) { this->text = text; }

rbtt::Interface::InterfaceSlider::InterfaceSlider(const proto::Slider& protoSlider) {
    this->text = protoSlider.text();
    this->min = protoSlider.min();
    this->max = protoSlider.max();
    this->interval = protoSlider.interval();
}

rbtt::Interface::InterfaceSlider::InterfaceSlider(const std::string text, const float min, const float max, const float interval) {
    this->text = text;
    this->min = min;
    this->max = max;
    this->interval = interval;
}
}
