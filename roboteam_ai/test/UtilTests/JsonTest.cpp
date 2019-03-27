#include <gtest/gtest.h>
#include "roboteam_ai/src/treeinterp/TreeInterpreter.h"
#include "roboteam_ai/src/treeinterp/BTFactory.h"

namespace rtt {
namespace ai {
namespace treeinterp {

TEST(JsonBasics, JsonTest)
{

    // Get the instance of the interpreter
    rtt::ai::treeinterp::TreeInterpreter interpreter;
    rtt::ai::treeinterp::JsonReader jsonReader;

    // Test file path utils
    std::vector<std::string> right1 = {"a", "b"};
    std::vector<std::string> left1 = jsonReader.split("a|b", '|');
    EXPECT_EQ(left1, right1);

    // Note: cant really test getFilePath without putting brain power into it.

    // Read the test JSON
    json testing = jsonReader.readJSON("jsonBasicTest");
    std::string typeTesting = typeid(testing).name();

    // See if the dummy JSON is correct
    EXPECT_EQ("granny", testing["name"]);
    EXPECT_EQ("nanny", testing["child"]["name"]);
    EXPECT_EQ("minnie", testing["child"]["child"]["name"]);

    // Read an actual project JSON check type
    json bigJson = jsonReader.readJSON("jsonBasicTestTwo");
    EXPECT_EQ(bigJson["name"], "rtt_jsonBasicTestTwo");
    std::string typeBigJson = typeid(bigJson).name();
    EXPECT_EQ(typeBigJson, typeTesting);

    // Get the small trees from the project manually check type
    auto trees = bigJson["data"]["trees"];
    std::string typeTrees = typeid(trees).name();
    EXPECT_EQ(typeTesting, typeTrees);

    // Get the first tree in the project check type and name
    json aTree = trees[0];
    std::string typeATree = typeid(trees).name();
    EXPECT_EQ(typeTesting, typeATree);
    EXPECT_EQ(aTree["scope"], "tree");
    EXPECT_EQ(aTree["title"], "ASimpleTree");

    // See if all of them are trees
    for (const json& current : trees) {
        EXPECT_EQ(current["scope"], "tree");
    }
}

}
}
}



