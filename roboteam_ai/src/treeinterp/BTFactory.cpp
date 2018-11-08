#include <utility>
#include <boost/filesystem.hpp>
//
// Created by baris on 04/10/18.
//

#include "BTFactory.h"

std::map<std::string, std::map<std::string, bt::BehaviorTree>> BTFactory::strategyRepo;


/// Update an entire project
void BTFactory::updateProject(std::string projectName) {
    auto project = interpreter.getTree(projectName);
    strategyRepo[projectName] = project;
}

/// Update one tree from a project
void BTFactory::updateTree(std::string projectName, std::string treeName) {

    auto tree = interpreter.getTreeWithID(projectName, treeName);
    strategyRepo[projectName][treeName] = tree;

}

/// Returns the Behaviour Tree Factory Singleton
BTFactory &BTFactory::getFactory() {
    static BTFactory instance;
    return instance;
}

/// Initiate the BTFactory
void BTFactory::init() {
    interpreter = TreeInterpreter::getInstance();

    // Interpret all the tactics and put them in tactics repo as Node::Ptr
    // TODO loop the actual folder
    tacticsRepo = interpreter.makeTactics("testTactic");



}
bt::BehaviorTree BTFactory::getTree(std::string treeName) {
    if (strategyRepo.find(treeName) != strategyRepo.end()) {
        return strategyRepo.find(treeName)->second;
    }
    ROS_ERROR("No Strategy by that name");
    return strategyRepo.end()->second;
}























