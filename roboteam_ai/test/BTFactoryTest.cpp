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

    // This will be the tree
    bt::BehaviorTree manualTree;

    bt::Sequence rootSeq;

    char cCurrentPath[FILENAME_MAX];

    if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath)))
    {
        std::cerr << "gh" << std::endl;
    }

    cCurrentPath[sizeof(cCurrentPath) - 1] = '\0'; /* not really required */

    printf ("The current working directory is %s", cCurrentPath);

    // Add root to the tree


}


