//
// Created by baris on 02/10/18.
//

#include <gtest/gtest.h>
#include "../src/treeinterp/BTFactory.h"

TEST(BT, BTTest) {
    // ===Let's build a BT manually!===
    // This  will be the tree
    bt::BehaviorTree manualTree;
    // TODO: test the functions of BTFactory once they can be implemented
}

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

    //Define file to test with and update the project to match it.
    std::string testProjectA = "TestStrategy";
    std::string testTreeA = "2d276c37-4ffc-4209-bc32-9fd2405a2ad6";
    std::string testTreeB = "77bc7cab-fe50-4754-9cd4-b80ef75cecf1";
    int testProjectSize = 5; // The amount of trees in the project

    //try updating a single Tree
    dummyFactory.updateTree(testProjectA, testTreeA);

    // Get the tree of that project
    bt::BehaviorTree::Ptr tree = dummyFactory.getTree(testProjectA);
    ASSERT_EQ(dummyFactory.strategyRepo.size(), 1);
    ASSERT_EQ(dummyFactory.strategyRepo.begin()->first, testTreeA);


    //try updating the whole project (automatically updates all trees with their ID's)
    dummyFactory.updateProject(testProjectA);

    // Check if all the trees are updated
    ASSERT_EQ(dummyFactory.strategyRepo.size(), testProjectSize);
//    ASSERT_NE(treeMapA.find(testTreeA), treeMapA.end());
//    ASSERT_NE(treeMapA.find(testTreeB), treeMapA.end());
//    ASSERT_EQ(treeMapA.find(testTreeA)->first, testTreeA);
//    ASSERT_EQ(treeMapA.find(testTreeB)->first, testTreeB);

}



