#include <utility>

#include <utility>

//
// Created by baris on 01/10/18.
//
/**
 * https://github.com/nlohmann/json
 */


#include "TreeInterpreter.h"

/// Return a TreeInterpreter singleton
TreeInterpreter &TreeInterpreter::getInstance() {
    static TreeInterpreter instance;
    return instance;
}

/// Returns a BehaviorTree from a given name
std::map<std::string, bt::BehaviorTree> TreeInterpreter::getProject(std::string name) {
    std::map<std::string, bt::BehaviorTree> result;

    // Read a project from file
    std::vector<json> project = readJSON(std::move(name));

    // Loop over all the trees in the project JSON and put them in the map
    // TODO: fix the names for the trees
    for (const json &tree : project) {
        bt::BehaviorTree currentTree = buildTreeFromJSON(tree);
        result.insert(std::pair<std::string, bt::BehaviorTree>("temp_name", currentTree));
    }
    return result;
}


bt::BehaviorTree TreeInterpreter::getTree(std::string projectName, std::string name) {

    // TODO: get the project and only parse and build one tree

    // return

}



/// Read JSON from a file
json TreeInterpreter::readJSON(std::string fileName) {

    // TODO: make relative path
    std::ifstream ifs("/home/baris/roboteamtwente/workspace/src/roboteam_ai/roboteam_ai/src/treeinterp/jsons/" + fileName + ".json");
    json bigJSON = json::parse(ifs);
    return bigJSON;
}

/// Parse from the project JSON small tree JSONs
std::vector<json> TreeInterpreter::parseSmallJSONs(json input) {

    std::vector<json> result;

    // First check if it is indeed a project
    if(input["data"]["scope"] == "project"){
        auto trees = input["data"]["trees"];

        // Loop and add all of the tress to the vector
        for (const json &current : trees) {
            result.push_back(current);
        }
    } else {
        std::cerr << "MURDER ME" << std::endl;
    }

    return result;
}

/// Build a BehaviorTree from a JSON object
bt::BehaviorTree TreeInterpreter::buildTreeFromJSON(json jsonTree) {



    return bt::BehaviorTree();
}

