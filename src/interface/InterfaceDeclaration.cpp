//
// Created by Dawid Kulikowski on 10/08/2021.
//

#include "interface/InterfaceDeclaration.h"
#include <roboteam_utils/Print.h>

InterfaceDeclaration::InterfaceDeclaration(const proto::UiOptionDeclaration &decl): defaultValue(InterfaceValue(decl.default_())) {
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
            RTT_ERROR("Can't instantiate interface declaration from incomplete proto message!");
            std::terminate();
            break;
        default:
            RTT_ERROR("Corrupted proto message!");
            std::terminate();
            break;
    }
}

InterfaceDeclaration::InterfaceDeclaration(const std::string path, const std::string description, const bool isMutable, const InterfaceValue defaultValue, const InterfaceOptions options): defaultValue(defaultValue) {
    this->path = path;
    this->description = description;
    this->isMutable = isMutable;
    this->options = options;
}
