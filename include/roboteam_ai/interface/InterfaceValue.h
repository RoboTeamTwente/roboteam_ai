//
// Created by Dawid Kulikowski on 08/08/2021.
//

#ifndef RTT_INTERFACEVALUE_H
#define RTT_INTERFACEVALUE_H

#include <string>
#include <variant>
#include <iostream>
#include <roboteam_proto/UiOptions.pb.h>
#include <nlohmann/json.hpp>
#include <exception>

namespace rtt::Interface {
struct InterfaceValue {
   public:
    InterfaceValue() = default;
    InterfaceValue(const proto::UiValue&);

    InterfaceValue(const int64_t v) : variant{v} {};
    InterfaceValue(const bool v) : variant{v} {};
    InterfaceValue(const float v) : variant{v} {};
    InterfaceValue(const std::string v) : variant{v} {};

    std::variant<int64_t, bool, float, std::string> variant;

    proto::UiValue toProto() const;
};
void from_json(const nlohmann::json& j, InterfaceValue& p);
void to_json(nlohmann::json& j, const InterfaceValue& p);

}  // namespace rbtt::Interface

#endif  // RTT_INTERFACEVALUE_H
