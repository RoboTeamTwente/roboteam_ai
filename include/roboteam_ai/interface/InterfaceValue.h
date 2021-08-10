//
// Created by Dawid Kulikowski on 08/08/2021.
//

#ifndef RTT_INTERFACEVALUE_H
#define RTT_INTERFACEVALUE_H

#include <string>
#include <roboteam_proto/UiOptions.pb.h>

struct InterfaceValue {
   public:
    InterfaceValue(const proto::UiValue&);

    InterfaceValue(const int64_t v): intValue{v} {};
    InterfaceValue(const bool v): boolValue{v} {};
    InterfaceValue(const float v): floatValue{v} {};
    InterfaceValue(const std::string v): textValue{v} {};

    std::optional<int64_t> intValue = std::nullopt;
    std::optional<bool> boolValue = std::nullopt;
    std::optional<float> floatValue = std::nullopt;
    std::optional<std::string> textValue = std::nullopt;
};

#endif  // RTT_INTERFACEVALUE_H
