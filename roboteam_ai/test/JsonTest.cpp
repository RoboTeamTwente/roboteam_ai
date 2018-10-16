//
// Created by baris on 02/10/18.
//

#include <gtest/gtest.h>
#include "../src/treeinterp/TreeInterpreter.h"
#include "../src/treeinterp/json.h"
#include "../src/treeinterp/JsonReader.h"

#include <typeinfo>


void print(const std::string &s) {
    std::cout << s << std::endl;
}

TEST(JsonBasics, JsonTest) {

    // Get the instance of the interpreter
    TreeInterpreter interpreter = TreeInterpreter::getInstance();

    JsonReader jsonReader;

    // Test file path utils
    std::vector<std::string> right1 = {"a", "b"};
    std::vector<std::string> left1 = jsonReader.split("a|b", '|');
    ASSERT_EQ(left1, right1);

    // Note: cant really test getFilePath without putting brain power into it.

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


    // ===== Test the reader functions ====

    auto btTesting = interpreter.getTreeWithID("bigjson", "d16f9751-a781-4fee-8570-7d88a207aef0"); // TODO: name files with the project names

    auto rootBTTesting = btTesting.GetRoot();

    ASSERT_TRUE(true);

}


TEST(TreeTest, JsonTest) {

    // TODO: implement these tests after conditions and skills are moved to this repository and they are put into the btBUILDER

    // Make node manually and with the function compare them
    // Make leaf manually and with function(name) and compare them
    // Build the sample.json tree manually and with the functions and compare them
    // Build the project bigjson.json and check a lot of things/see if things fail

}





