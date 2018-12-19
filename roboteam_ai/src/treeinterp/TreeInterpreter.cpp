//
// Created by baris on 01/10/18.
//

#include "TreeInterpreter.h"

/// Return a TreeInterpreter singleton
TreeInterpreter &TreeInterpreter::getInstance() {
    static TreeInterpreter instance;
    return instance;
}

/// Returns a BehaviorTree from a given name
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

    // Return object
    bt::Node::Ptr node;

    // It might be a Role (Special Case)
    if (nodeJSON["title"] == "Role") {
        bt::Role::Ptr tempNode = std::make_shared<bt::Role>(nodeJSON["name"]);
        tempNode->globalBB = globalBlackBoard;
        tempNode->properties = propertyParser.parse(nodeJSON);
        node = tempNode;
        // Build and actual normal node
    }
    else {
        bt::Node::Ptr tempNode = makeNonLeafNode(nodeJSON["title"]);
        tempNode->globalBB = globalBlackBoard;
        tempNode->properties = propertyParser.parse(nodeJSON);
        node = tempNode;
    };

    if (! jsonReader.checkIfKeyExists("children", nodeJSON)) {
        ROS_ERROR("Well this is a non leaf node without children and this should never happen. Like really never");
    }

    // Get the children in the node
    for (std::string currentChildID : nodeJSON["children"]) {
        auto currentChild = tree["nodes"][currentChildID];

        node->addChild(buildNode(currentChild, tree, globalBlackBoard));
    }
    return node;
}

/// Makes a non leaf node
bt::Node::Ptr TreeInterpreter::makeNonLeafNode(std::string name) {

    bt::Node::Ptr node = Switches::nonLeafSwitch(std::move(name));

    // TODO maybe process the node here a little otherwise this is a very redundant function
    return node;
}

/// Returns if there is any element in a json called "child" or "children"
bool TreeInterpreter::isLeaf(json jsonTree) {
    bool hasChild = jsonReader.checkIfKeyExists("child", jsonTree); // legacy
    bool hasChildren = jsonReader.checkIfKeyExists("children", jsonTree);
    return ! (hasChild || hasChildren);
}

/// Make a leaf node depending on the name of the node
bt::Leaf::Ptr TreeInterpreter::makeLeafNode(json jsonLeaf) {

    if (jsonLeaf["title"] == "Tactic") {
        auto properties = propertyParser.parse(jsonLeaf);
        auto node = tacticSwitch(jsonLeaf["name"], properties);
        node->addChild(tactics.find(jsonLeaf["name"])->second);
        return node;
    }

    bt::Blackboard::Ptr properties = propertyParser.parse(jsonLeaf);

    rtt::ai::Skill::Ptr skill = Switches::leafSwitch(jsonLeaf["title"], properties);

    return skill;

}

/// Reads all the tactics from one JSON project, puts them in a local repositopry and returns them
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

    bt::Node::Ptr node = Switches::tacticSwitch(name, std::move(properties));
    node->addChild(tactics.find(name)->second);
    return node;
}








