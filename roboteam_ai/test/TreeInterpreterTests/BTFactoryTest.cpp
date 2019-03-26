
#include <gtest/gtest.h>
#include "roboteam_ai/src/treeinterp/BTFactory.h"

TEST (BT, JsonEditor) {
    rtt::ai::treeinterp::BTFactory dummyFactory;
    std::string testProject = "sample";
    std::string testTree = "sAmPleNamE";
    rtt::ai::treeinterp::JsonReader pathReader;

    pathReader.editJSON(testProject, testTree, "description", "TESTING");
    json readJson = pathReader.readJSON(testProject);

    EXPECT_EQ(readJson["data"]["trees"][0]["description"], "TESTING");
    pathReader.editJSON(testProject, testTree, "description", "A");

    readJson = pathReader.readJSON(testProject);
    EXPECT_EQ(readJson["data"]["trees"][0]["description"], "A");

    pathReader.editJSON(testProject, testTree, "description", "");
    readJson = pathReader.readJSON(testProject);
    EXPECT_EQ(readJson["data"]["trees"][0]["description"], "");
}


// Warning: tests depend on functioning of JsonTest and BTtests!!
TEST(BT, BasicFactoryTest) {
    rtt::ai::treeinterp::BTFactory dummyFactory;
    dummyFactory.init();
    std::string trace = "";

    bt::BehaviorTree::Ptr strategyTree = dummyFactory.getTree("haltStrategy");
    bt::Node::Ptr node = strategyTree->GetRoot();

    // add the first node
    trace += node->node_name() + "-";

    // loop recursively through nodes
    // we assume that there is only one child
    while (! node->getChildren().empty()) {
        node = node->getChildren().at(0);
        trace += node->node_name() + "-";
    }
    EXPECT_EQ(trace, "UntilSuccess-haltTactic-ParallelSequence-halt0-UntilSuccess-Halt-");
}
