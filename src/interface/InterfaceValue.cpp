//
// Created by Dawid Kulikowski on 08/08/2021.
//
#include "interface/InterfaceValue.h"

namespace rbtt::Interface {

InterfaceValue::InterfaceValue(const proto::UiValue& val) {
    switch (val.value_case()) {
        case proto::UiValue::ValueCase::kBoolValue:
            this->variant = val.bool_value();
            break;
        case proto::UiValue::ValueCase::kFloatValue:
            this->variant = val.float_value();
            break;
        case proto::UiValue::ValueCase::kIntegerValue:
            this->variant = val.integer_value();
            break;
        case proto::UiValue::ValueCase::kTextValue:
            this->variant = val.text_value();
            break;
        case proto::UiValue::ValueCase::VALUE_NOT_SET:
            RTT_ERROR("[Interface] Interface value not set (corrupted proto message)");
            std::terminate();
            break;
        default:
            RTT_ERROR("[Interface] Interface value unknown (corrupted proto message)");
            std::terminate();
            break;
    }
}

void to_json(nlohmann::json& j, const InterfaceValue& p) {
    if (auto* intVal = std::get_if<int64_t>(&p.variant)) {
        j = nlohmann::json{"int", *intVal};
    } else if (auto* boolVal = std::get_if<bool>(&p.variant)) {
        j = nlohmann::json{"bool", *boolVal};
    } else if (auto* floatVal = std::get_if<float>(&p.variant)) {
        j = nlohmann::json{"float", *floatVal};
    } else if (auto* stringVal = std::get_if<std::string>(&p.variant)) {
        j = nlohmann::json{"string", *stringVal};
    } else {
        throw std::logic_error{"Invalid state of InterfaceValue during serialization!"};
    }
}

void from_json(const nlohmann::json& j, InterfaceValue& p) {
    if (j.contains("int")) {
        p.variant = j.at("int").get<int>();
    } else if (j.contains("bool")) {
        p.variant = j.at("bool").get<bool>();
    } else if (j.contains("float")) {
        p.variant = j.at("float").get<float>();
    } else if (j.contains("string")) {
        p.variant = j.at("string").get<std::string>();
    } else {
        throw std::domain_error{"JSON representation of InterfaceValue contains unknown type of value!"};
    }
}
}