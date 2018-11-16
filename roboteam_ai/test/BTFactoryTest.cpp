#include <gtest/gtest.h>
#include "../src/treeinterp/BTFactory.h"
#include "../src/bt/tactics/DemoTactic.h"

TEST (BT, JsonEditor) {
    BTFactory dummyFactory = BTFactory::getFactory();
    std::string testProject = "sample";
    std::string testTree = "sAmPleNamE";
    JsonReader pathReader;

    pathReader.editJSON(testProject, testTree, "description", "TESTING");
    json readJson = pathReader.readJSON(testProject);

    ASSERT_EQ(readJson["data"]["trees"][0]["description"], "TESTING");
    pathReader.editJSON(testProject, testTree, "description", "A");

    readJson = pathReader.readJSON(testProject);
    ASSERT_EQ(readJson["data"]["trees"][0]["description"], "A");

    pathReader.editJSON(testProject, testTree, "description", "");
    readJson = pathReader.readJSON(testProject);
    ASSERT_EQ(readJson["data"]["trees"][0]["description"], "");
}


// Warning: tests depend on functioning of JsonTest and BTtests!!
TEST(BT, BasicFactoryTest) {
    BTFactory dummyFactory = BTFactory::getFactory();
    dummyFactory.init();
//    bt::BehaviorTree::Ptr strategyTree = dummyFactory.getTree("DemoStrategy");
//    ASSERT_EQ(strategyTree->GetRoot()->node_name(), "Repeater");
//    ASSERT_EQ(strategyTree->GetRoot()->getChildren().size(), 1);
//    bt::Node::Ptr hopefullyARepeater = strategyTree->GetRoot();
//    ASSERT_EQ(hopefullyARepeater->getChildren().size(), 1);
//    bt::Node::Ptr hopefullyADemoTactic = hopefullyARepeater->getChildren().at(0);
//    ASSERT_EQ(hopefullyADemoTactic->getChildren().size(), 1);
//    ASSERT_EQ(hopefullyADemoTactic->node_name(), "DemoTactic");

    std::string trace = "";

    bt::BehaviorTree::Ptr strategyTree = dummyFactory.getTree("DemoStrategy");
    bt::Node::Ptr node = strategyTree->GetRoot();

    // add the first node
    trace += node->node_name() + "-";

    // loop recursively through nodes
    // we assume that there is only one child
    while (!node->getChildren().empty()) {
        node = node->getChildren().at(0);
        trace += node->node_name() + "-";
    }
    ASSERT_EQ(trace, "Repeater-Demo Tactic-testRole-MemSequence-GoToPos-");
}
