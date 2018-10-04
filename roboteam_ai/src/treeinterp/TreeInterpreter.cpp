#include <utility>

#include <utility>

//
// Created by baris on 01/10/18.
//
/**
 * https://github.com/nlohmann/json
 */


#include "TreeInterpreter.h"
#include <unistd.h>

/// Return a TreeInterpreter singleton
TreeInterpreter &TreeInterpreter::getInstance() {
    static TreeInterpreter instance;
    return instance;
}

/// Build a BehaviorTree from a JSON object
bt::BehaviorTree TreeInterpreter::buildTreeFromJson(json json) {
    return bt::BehaviorTree();
}

/// Returns a BehaviorTree from a given name
std::map<std::string, bt::BehaviorTree> TreeInterpreter::getTrees(std::string name) {
    std::map<std::string, bt::BehaviorTree> result;

    // Read a project from file
    std::vector<json> project = readJsons(std::move(name));

    // Loop over all the trees in the project JSON and put them in the map
    // TODO: fix the names for the trees
    for (const json &tree : project) {
        bt::BehaviorTree currentTree = buildTreeFromJson(tree);
        result.insert(std::pair<std::string, bt::BehaviorTree>("temp_name", currentTree));
    }
    return result;
}

/// Read JSON from a file
std::vector<json> TreeInterpreter::readJsons(std::string fileName) {

    // TODO: make relative path
    std::ifstream ifs("/home/baris/roboteamtwente/workspace/src/roboteam_ai/roboteam_ai/src/treeinterp/jsons/" + fileName + ".json");
    json bigJSON = json::parse(ifs);
    std::vector<json> smallJsons = parseSmallJsons(bigJSON);
    return smallJsons;
}

/// Parse from the project JSON small tree JSONs
std::vector<json> TreeInterpreter::parseSmallJsons(json input) {

    std::vector<json> result;

    //TODO: break the big JSON into smaller JSONs

    return result;
}

