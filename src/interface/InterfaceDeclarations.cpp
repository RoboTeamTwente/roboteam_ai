//
// Created by Dawid Kulikowski on 10/08/2021.
//
#include "interface/InterfaceDeclarations.h"
#include <algorithm>
#include <vector>

void InterfaceDeclarations::_unsafeAddDeclaration(const InterfaceDeclaration decl) {
    this->decls.push_back(decl);
}

void InterfaceDeclarations::_unsafeRemoveDeclaration(const std::string path) {
    this->decls.erase(std::remove_if(this->decls.begin(), this->decls.end(), [&](const auto& item){
        return (item.path == path);
    }), this->decls.end());
}

std::optional<InterfaceDeclaration> InterfaceDeclarations::_unsafeGetDeclaration(const std::string path) const {
    auto found = std::find_if(this->decls.begin(), this->decls.end(), [&](const auto& item){
        return item.path == path;
    });

    return found != this->decls.end() ? std::make_optional(*found) : std::nullopt;
}

void InterfaceDeclarations::addDeclaration(const InterfaceDeclaration decl) {
    std::scoped_lock(this->mtx);

    this->_unsafeAddDeclaration(decl);
}

void InterfaceDeclarations::removeDeclaration(const std::string path) {
    std::scoped_lock(this->mtx);

    this->_unsafeRemoveDeclaration(path);
}

std::vector<InterfaceDeclaration> InterfaceDeclarations::getDeclarations() const {
    std::scoped_lock(this->mtx);

    return this->decls;
}

std::optional<InterfaceDeclaration> InterfaceDeclarations::getDeclaration(const std::string path) const {
    std::scoped_lock(this->mtx);

    return this->_unsafeGetDeclaration(path);
}

void InterfaceDeclarations::handleData(const proto::UiOptionDeclarations &decls) {
    // TODO: Parse proto

}
