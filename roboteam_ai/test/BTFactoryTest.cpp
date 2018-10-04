//
// Created by baris on 02/10/18.
//

#include <gtest/gtest.h>
#include "../src/treeinterp/TreeInterpreter.h"
#include "../src/treeinterp/json.h"



TEST(Tree, JsonTest) {

    // Get the instance of the interpreter
    TreeInterpreter interpreter = TreeInterpreter::getInstance();

    // Read the test JSON
    json testing = interpreter.readJsons("test");

    ASSERT_EQ("granny", testing["name"]);
    ASSERT_EQ("nanny", testing["child"]["name"]);
    ASSERT_EQ("minnie", testing["child"]["child"]["name"]);

}


