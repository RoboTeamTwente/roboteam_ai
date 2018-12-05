#include <gtest/gtest.h>
#include "../src/treeinterp/TreeInterpreter.h"

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
}

TEST(TreeTest, JsonTest) {

    // TODO: implement these tests after conditions and skills are moved to this repository and they are put into the btBUILDER

    // Make node manually and with the function compare them
    // Make leaf manually and with function(name) and compare them
    // Build the sample.json tree manually and with the functions and compare them
    // Build the project bigjson.json and check a lot of things/see if things fail

}





