//
// Created by baris on 02/10/18.
//

#include <gtest/gtest.h>
#include "../src/treeinterp/BTFactory.h"
#include "../src/bt/Composite.hpp"
#include "../src/bt/composites/Sequence.hpp"
#include "../src/bt/Leaf.hpp"

#include <stdio.h>  /* defines FILENAME_MAX */
#ifdef WINDOWS
#include <direct.h>
    #define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif



TEST(BT, BTTest) {

    // ===Let's build a BT manually!===

    // This  will be the tree
    bt::BehaviorTree manualTree;
    // TODO: test the functions of BTFactory once they can be implemented

}

TEST(BT,SimpleFactoryTest){
    BTFactory dummyFactory=BTFactory::getFactory();

    //Define file to test with and update the project to match it.
    std::string testCaseA="bigjson";
    dummyFactory.updateProject(testCaseA);

    // Get the tree of that project.
    std::map<std::string, bt::BehaviorTree> treeMapA=dummyFactory.getProject(testCaseA);

    //initialize the BTFactory. This SHOULD work.
    dummyFactory.init();
}


