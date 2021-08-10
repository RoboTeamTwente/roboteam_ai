//
// Created by Dawid Kulikowski on 08/08/2021.
//
#include "interface/InterfaceValue.h"

InterfaceValue::InterfaceValue(const proto::UiValue& val) {
    intValue = val.has_integer_value() ? std::make_optional(val.integer_value()) : std::nullopt;
    floatValue = val.has_float_value() ? std::make_optional(val.float_value()) : std::nullopt;
    boolValue = val.has_bool_value() ? std::make_optional(val.bool_value()) : std::nullopt;
    textValue = val.has_text_value() ? std::make_optional(val.text_value()) : std::nullopt;
}
