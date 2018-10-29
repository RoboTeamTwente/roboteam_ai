//
// Created by baris on 02/10/18.
//

#include <gtest/gtest.h>
#include "../src/treeinterp/BTFactory.h"
#include "../src/bt/Composite.hpp"
#include "../src/bt/composites/Sequence.hpp"
#include "../src/bt/Leaf.hpp"
#include "../src/treeinterp/JsonReader.h"





TEST(BT, BTTest) {

    // ===Let's build a BT manually!===

    // This  will be the tree
    bt::BehaviorTree manualTree;
    // TODO: test the functions of BTFactory once they can be implemented

}

TEST (BT,JsonEditor){
    BTFactory dummyFactory=BTFactory::getFactory();
    std::string testProject="sample";
    std::string testTree="sAmPleNamE";
    JsonReader pathReader;

    pathReader.editJSON(testProject,testTree,"description","TESTING");
    json readJson =pathReader.readJSON(testProject);

    ASSERT_EQ(readJson["data"]["trees"][0]["description"],"TESTING");
    pathReader.editJSON(testProject,testTree,"description","A");

    readJson=pathReader.readJSON(testProject);
    ASSERT_EQ(readJson["data"]["trees"][0]["description"],"A");

    pathReader.editJSON(testProject,testTree,"description","");
    readJson=pathReader.readJSON(testProject);
    ASSERT_EQ(readJson["data"]["trees"][0]["description"],"");
}

// Warning: tests depend on functioning of JsonTest and BTtests!!
TEST(BT,BasicFactoryTest){

    BTFactory dummyFactory=BTFactory::getFactory();

    //Define file to test with and update the project to match it.
    std::string testProjectA="sampleTest";
    std::string testTreeA="2d276c37-4ffc-4209-bc32-9fd2405a2ad6";
    std::string testTreeB="77bc7cab-fe50-4754-9cd4-b80ef75cecf1";
    int testProjectSize=26; // The amount of trees in the project
    //try updating a single Tree
    dummyFactory.updateTree(testProjectA,testTreeA);

    // Get the tree of that project
    std::map<std::string, bt::BehaviorTree> treeMapA=dummyFactory.getProject(testProjectA);

    // CHeck if it only updated one Tree and the right Tree
    ASSERT_EQ(treeMapA.size(),1);
    ASSERT_EQ(treeMapA.find(testTreeA)->first,testTreeA);
    ASSERT_NE(treeMapA.find(testTreeA),treeMapA.end());
    ASSERT_EQ(treeMapA.find(testTreeB),treeMapA.end());

    treeMapA.clear();

    //try updating the whole project (automatically updates all trees with their ID's)
    dummyFactory.updateProject(testProjectA);

    // get the tree again:
    treeMapA=dummyFactory.getProject(testProjectA);


    // Check if all the trees are updated
    ASSERT_EQ(treeMapA.size(),testProjectSize);
    ASSERT_NE(treeMapA.find(testTreeA),treeMapA.end());
    ASSERT_NE(treeMapA.find(testTreeB),treeMapA.end());
    ASSERT_EQ(treeMapA.find(testTreeA)->first,testTreeA);
    ASSERT_EQ(treeMapA.find(testTreeB)->first,testTreeB);



    //initialize the BTFactory. This updates all the projects in it
    //dummyFactory.init();
}
TEST(BT,UpdateDuringRuntime){
    BTFactory dummyFactory=BTFactory::getFactory();
    std::string testProjectA="sampleTest";
    std::string testTreeA="2d276c37-4ffc-4209-bc32-9fd2405a2ad6";
    std::string testTreeB="d16f9751-a781-4fee-8570-7d88a207aef0";
    //Initial update of the project
    dummyFactory.updateProject(testProjectA);


    // Now we will test if we edit a tree during runtime, and then update it, that it is actually updated.
    JsonReader pathReader;
    // Fields we are editing and their original values
    std::string fieldToEdit="root";
    std::string newValueA="f541478e-e705-46e7-8801-155dd28bf4eb";
    std::string originalA="5462f6e1-8a5d-4d2e-82c4-8739ac16ca04";

    std::string newValueB="28b9ae61-7734-4492-8a43-12ed385249b2";
    std::string originalB="05290a9f-6795-431d-8999-140af520edef";
    pathReader.editJSON(testProjectA,testTreeA,fieldToEdit,originalA);
    pathReader.editJSON(testProjectA,testTreeB,fieldToEdit,originalB);

    //first we test if the root is equal to the original for both
    std::map<std::string, bt::BehaviorTree> testMap=dummyFactory.getProject(testProjectA);
    bt::BehaviorTree treeA=testMap[testTreeA];
    bt::BehaviorTree treeB=testMap[testTreeB];

    // The tree of A has no null pointer already

    ASSERT_NE(treeA.GetRoot(), nullptr);
    //Now we change both values in the .json file
    pathReader.editJSON(testProjectA,testTreeA,fieldToEdit,newValueA);
    pathReader.editJSON(testProjectA,testTreeB,fieldToEdit,newValueB);
    // Update ONLY treeA
    dummyFactory.updateTree(testProjectA,testTreeA);

    testMap=dummyFactory.getProject(testProjectA);
    bt::BehaviorTree treeAchanged=testMap[testTreeA];
    bt::BehaviorTree treeBchanged=testMap[testTreeB];

    // The pointer in A changed but the pointer of treeB did not change.
    ASSERT_EQ(treeAchanged.GetRoot(),nullptr);
    ASSERT_EQ(treeBchanged.GetRoot(),treeB.GetRoot());

    dummyFactory.updateProject(testProjectA);

    testMap=dummyFactory.getProject(testProjectA);
    bt::BehaviorTree treeAchangedtwice=testMap[testTreeA];
    bt::BehaviorTree treeBchangedtwice=testMap[testTreeB];

    ASSERT_EQ(treeAchangedtwice.GetRoot(), nullptr);
    ASSERT_NE(treeBchangedtwice.GetRoot(),treeBchanged.GetRoot());
    ASSERT_NE(treeBchangedtwice.GetRoot(),nullptr);
    //Change back the .json file and check this
    pathReader.editJSON(testProjectA,testTreeA,fieldToEdit,originalA);
    pathReader.editJSON(testProjectA,testTreeB,fieldToEdit,originalB);

}


