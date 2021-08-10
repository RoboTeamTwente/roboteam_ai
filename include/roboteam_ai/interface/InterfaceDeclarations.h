//
// Created by Dawid Kulikowski on 09/08/2021.
//

#ifndef RTT_INTERFACEDECLARATIONS_H
#define RTT_INTERFACEDECLARATIONS_H

#include <roboteam_proto/UiOptions.pb.h>
#include "InterfaceDeclaration.h"

class InterfaceDeclarations {
   private:
    std::vector<InterfaceDeclaration> decls = {};

    mutable std::mutex mtx;

    void _unsafeAddDeclaration(const InterfaceDeclaration);
    void _unsafeRemoveDeclaration(const std::string);

    std::optional<InterfaceDeclaration> _unsafeGetDeclaration(const std::string) const;
   public:
    void addDeclaration(const InterfaceDeclaration);
    void removeDeclaration(const std::string);

    std::vector<InterfaceDeclaration> getDeclarations() const;
    std::optional<InterfaceDeclaration> getDeclaration(const std::string) const;


    void handleData(const proto::UiOptionDeclarations&);



};

#endif  // RTT_INTERFACEDECLARATIONS_H
