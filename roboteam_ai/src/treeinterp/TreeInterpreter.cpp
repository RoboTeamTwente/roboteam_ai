
//
// Created by baris on 01/10/18.
//
/**
 * https://github.com/nlohmann/json
 */


#include "TreeInterpreter.h"
#include "../bt/tactics/DemoTactic.h"
#include "../bt/Role.h"

// all skills..
#include "../skills/GoToPos.h"
#include "../skills/Kick.h"


/// Return a TreeInterpreter singleton
TreeInterpreter &TreeInterpreter::getInstance() {
    static TreeInterpreter instance;
    return instance;
}

/// Returns a BehaviorTree from a given name TESTED
std::map<std::string, bt::BehaviorTree::Ptr> TreeInterpreter::getTrees(std::string name) {
    std::map<std::string, bt::BehaviorTree::Ptr> result;

    auto project = jsonReader.readJSON(std::move(name));

    for (const json &tree : project["data"]["trees"]) {
        std::string treeName = tree["title"];
        bt::BehaviorTree::Ptr currentTree = buildTreeFromJSON(tree);
        result.insert(std::pair<std::string, bt::BehaviorTree::Ptr>(treeName, currentTree));
    }
    return result;
}


/// Parse from the project JSON small tree JSONs
std::vector<json> TreeInterpreter::parseSmallJSONs(json input) {

    std::vector<json> result;

    // First check if it is indeed a project
    if (input["data"]["scope"] == "project") {
        auto trees = input["data"]["trees"];

        // Loop and add all of the trees to the vector
        for (const json &current : trees) {
            result.push_back(current);
        }
    }
    else {
        std::cerr << "The JSON tree is not a project!" << std::endl;
    }

    return result;
}

/// Build a BehaviorTree from a JSON object
bt::BehaviorTree::Ptr TreeInterpreter::buildTreeFromJSON(json jsonTree) {

    // ID of the root
    std::string rootID = jsonTree["root"];


    // Build the tree from the root
    bt::BehaviorTree::Ptr behaviorTree = std::make_shared<bt::BehaviorTree>();
    bt::Blackboard::Ptr globalBB = propertyParser.parse(jsonTree);
    behaviorTree->setProperties(globalBB);

    auto rootNode = TreeInterpreter::buildNode(jsonTree["nodes"][rootID], jsonTree, globalBB);
    behaviorTree->SetRoot(rootNode);
    return behaviorTree;
}

/// Builds nodes recursively from json objects
bt::Node::Ptr TreeInterpreter::buildNode(json nodeJSON, json tree, bt::Blackboard::Ptr globalBlackBoard) {

    // Dealing with a leaf, the properties are set internally
    if (TreeInterpreter::isLeaf(nodeJSON)) {

        bt::Leaf::Ptr leaf;
        leaf = TreeInterpreter::makeLeafNode(nodeJSON);
        leaf->globalBB = globalBlackBoard;
        return leaf;
    }

    bt::Node::Ptr node;

    // It might be a Role
    if (nodeJSON["title"] == "Role") {
        bt::Role::Ptr tempNode = std::make_shared<bt::Role>(nodeJSON["name"]);
        tempNode->globalBB = globalBlackBoard;
        tempNode->properties = propertyParser.parse(nodeJSON);
        node = tempNode;
    } else {
        bt::Node::Ptr tempNode = makeNonLeafNode(nodeJSON["name"]);
        tempNode->globalBB = globalBlackBoard;
        tempNode->properties = propertyParser.parse(nodeJSON);
        node = tempNode;
    };

    buildTree(nodeJSON, tree, globalBlackBoard, node);

    return node;
}
void TreeInterpreter::buildTree(const json &nodeJSON, const json &tree, const bt::Blackboard::Ptr &globalBlackBoard,
        bt::Node::Ptr &node) {

    // One child
    if (jsonReader.checkIfKeyExists("child", nodeJSON)) {
        std::string childID = nodeJSON["child"];
        auto child = tree["nodes"][childID];

        node->AddChild(buildNode(child, tree, globalBlackBoard));
    }

    // Multiple children
    else {

        for (std::string currentChildID : nodeJSON["children"]) {
            auto currentChild = tree["nodes"][currentChildID];

            node->AddChild(buildNode(currentChild, tree, globalBlackBoard));
        }
    }
}

/// Makes a non leaf node
bt::Node::Ptr TreeInterpreter::makeNonLeafNode(std::string name) {

    // C++ doesnt like switches with strings :(

    bt::Node::Ptr node;

    // TODO: check the namings from the bt module and fix/add them here

    // Some of the naming is archaic, but we need it here.
    if (name == "MemSelector" || name == "Selector" || name == "Priority" || name == "MemPriority") {
        node = std::make_shared<bt::MemSelector>();
    }
    else if (name == "MemSequence") {
        node = std::make_shared<bt::MemSequence>();
    }
    else if (name == "ParallelSequence") {
        node = std::make_shared<bt::ParallelSequence>();
    }
    else if (name == "Selector") {
        node = std::make_shared<bt::Selector>();
    }
    else if (name == "Sequence" || name == "ParallelTactic") { // TODO: parallel here?
        node = std::make_shared<bt::Sequence>();
    }
    else if (name == "Failer") {
        node = std::make_shared<bt::Failer>();
    }
    else if (name == "Inverter") {
        node = std::make_shared<bt::Inverter>();
    }
    else if (name == "Repeat") {
        node = std::make_shared<bt::Repeater>();
    }
    else if (name == "Succeeder") {
        node = std::make_shared<bt::Succeeder>();
    }
    else if (name == "UntilFail") {
        node = std::make_shared<bt::UntilFail>();
    }
    else if (name == "UntilSuccess" || name == "RepeatUntilSuccess") {
        node = std::make_shared<bt::UntilSuccess>();
    }
    else {
        std::cerr << "Node name with: " + name << std::endl;
    }
    return node;

}

/// Returns if there is any element in a json called "child" or "children"
bool TreeInterpreter::isLeaf(json jsonTree) {
    bool hasChild = jsonReader.checkIfKeyExists("child", jsonTree);
    bool hasChildren = jsonReader.checkIfKeyExists("children", jsonTree);
    return ! (hasChild || hasChildren);
}

/// Make a leaf node depending on the name of the node
bt::Leaf::Ptr TreeInterpreter::makeLeafNode(json jsonLeaf) {

    if (jsonLeaf["title"] == "Tactic") {
        auto properties = propertyParser.parse(jsonLeaf);
        auto node = tacticSwitch(jsonLeaf["name"], properties);
        node->AddChild(tactics.find(jsonLeaf["name"])->second);
        return node;

    }
    rtt::ai::Skill::Ptr skill;
    std::string name = jsonLeaf["title"];
    bt::Blackboard::Ptr properties = propertyParser.parse(jsonLeaf);

    if (name == "Dummy") {
        // TODO: after importing the leaf subclasses make a switch here
        // leaf = make_some_condition_or_something()
    }
    else if (name == "GoToPos") {
        skill = std::make_shared<rtt::ai::GoToPos>(name, properties);
    }
    else if (name == "Kick") {
        skill = std::make_shared<rtt::ai::Kick>(name, properties);
    }
    else {
        skill = std::make_shared<rtt::ai::GoToPos>(name, properties);
    }

    return skill;

}
std::map<std::string, bt::Node::Ptr> TreeInterpreter::makeTactics(std::string fileName, bt::Blackboard::Ptr globalBB) {
    json tacticJson = jsonReader.readJSON("tactics/" + fileName);
    std::map<std::string, bt::Node::Ptr> resultMap;
    for (auto tactic : tacticJson["data"]["trees"]) {
        std::string rootID = tactic["root"];
        bt::Node::Ptr buildingNode = TreeInterpreter::buildNode(tactic["nodes"][rootID], tactic, globalBB);
        resultMap.insert(std::pair<std::string, bt::Node::Ptr>(tactic["title"], buildingNode));
        tactics.insert(std::pair<std::string, bt::Node::Ptr>(tactic["title"], buildingNode));
    }
    return resultMap;
}
bt::Node::Ptr TreeInterpreter::tacticSwitch(std::string name, bt::Blackboard::Ptr properties) {
    if (name == "DemoTactic") {
        auto node = std::make_shared<bt::DemoTactic>("DemoTactic", properties);
        node->AddChild(tactics.find("DemoTactic")->second);
        return node;
    }
    auto node = std::make_shared<bt::DemoTactic>("name", properties);
    return node;
    // TODO: populate
}








