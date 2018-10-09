//
// Created by baris on 02/10/18.
//

#include <gtest/gtest.h>
#include "../src/treeinterp/TreeInterpreter.h"
#include "../src/treeinterp/json.h"
#include <typeinfo>


void print(const std::string &s) {
    std::cout << s << std::endl;
}

TEST(Tree, JsonTest) {

    // Get the instance of the interpreter
    TreeInterpreter interpreter = TreeInterpreter::getInstance();

    JsonReader jsonReader;

    // Read the test JSON
    json testing = jsonReader.readJSON("test");
    std::string typeTesting = typeid(testing).name();

    // See if the dummy JSON is correct
    ASSERT_EQ("granny", testing["name"]);
    ASSERT_EQ("nanny", testing["child"]["name"]);
    ASSERT_EQ("minnie", testing["child"]["child"]["name"]);

    // Read an actual project JSON check type
    json bigJson = jsonReader.readJSON("bigjson");
    ASSERT_EQ(bigJson["name"], "rtt_jim");
    std::string typeBigJson = typeid(bigJson).name();
    ASSERT_EQ(typeBigJson, typeTesting);

    // Get the small trees from the project manually check type
    auto trees = bigJson["data"]["trees"];
    std::string typeTrees = typeid(trees).name();
    ASSERT_EQ(typeTesting, typeTrees);

    // Get the first tree in the project check type and name
    json aTree = trees[0];
    std::string typeATree = typeid(trees).name();
    ASSERT_EQ(typeTesting, typeATree);
    ASSERT_EQ(aTree["scope"], "tree");
    ASSERT_EQ(aTree["title"], "DefenderRoleStop");

    // See if all of them are trees
    for (const json &current : trees) {
        ASSERT_EQ(current["scope"], "tree");
    }

    // Test the actual function that does this
    auto newTrees = interpreter.parseSmallJSONs(bigJson);

    // See if all of them are trees
    for (const json &current : newTrees) {
        ASSERT_EQ(current["scope"], "tree");
    }

    // Compare the manual and the automated
    ASSERT_EQ(trees, newTrees);
    

}




