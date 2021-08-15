//
// Created by Dawid Kulikowski on 09/08/2021.
//

#ifndef RTT_INTERFACEDECLARATIONS_H
#define RTT_INTERFACEDECLARATIONS_H

#include <roboteam_proto/UiOptions.pb.h>

#include <nlohmann/json.hpp>

#include "InterfaceController.h"
#include "InterfaceDeclaration.h"

namespace rbtt::Interface {
class InterfaceDeclarations {
   private:
    std::vector<InterfaceDeclaration> decls = {};

    std::weak_ptr<Interface> iface;

    mutable std::mutex mtx;

    void _unsafeAddDeclaration(InterfaceDeclaration);
    void _unsafeRemoveDeclaration(std::string);

    std::optional<InterfaceDeclaration> _unsafeGetDeclaration(std::string) const;

   public:
    InterfaceDeclarations() = default;
    InterfaceDeclarations(const proto::UiOptionDeclarations& pdecls) {this->handleData(pdecls);}
    explicit InterfaceDeclarations(const std::vector<InterfaceDeclaration> vec, const std::weak_ptr<Interface> piface) : decls(vec), iface(piface){};

    void addDeclaration(InterfaceDeclaration);
    void removeDeclaration(std::string);

    std::vector<InterfaceDeclaration> getDeclarations() const;
    std::optional<InterfaceDeclaration> getDeclaration(std::string) const;

    void handleData(const proto::UiOptionDeclarations&);

    proto::UiOptionDeclarations toProtoMessage();
};

}
#endif  // RTT_INTERFACEDECLARATIONS_H
