//
// Created by baris on 04/10/18.
//

#include "BTFactory.h"

/// Return a map of tree names and trees that belong to one project
std::map<std::string, bt::BehaviorTree> BTFactory::getProject(std::string projectName) {
    return std::map<std::string, bt::BehaviorTree>();
}

/// Update an entire project
void BTFactory::updateProject(std::string projectName) {
    // TODO: re-read the JSON project

}

/// Update one tree from a project
void BTFactory::updateTree(std::string projectName, std::string treeName) {
    //TODO: only update the tree
}

/// Returns the Behaviour Tree Factory Singelton
BTFactory &BTFactory::getFactory() {
    static BTFactory instance;
    return instance;
}

/// Initiate the BTFactory
void BTFactory::init() {
    interpreter = TreeInterpreter::getInstance();
    // TODO: turn all of the jsons into BTs

}
