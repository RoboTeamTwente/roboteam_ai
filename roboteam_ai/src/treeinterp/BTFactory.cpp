//
// Created by baris on 04/10/18.
//

#include "BTFactory.h"

std::map<std::string, bt::BehaviorTree> BTFactory::getProject(std::string projectName) {
    return std::map<std::string, bt::BehaviorTree>();
}

void BTFactory::updateProject(std::string projectName) {

}

void BTFactory::updateTree(std::string projectName, std::string treeName) {

}

BTFactory &BTFactory::getFactory() {
    static BTFactory instance;
    return instance;
}
