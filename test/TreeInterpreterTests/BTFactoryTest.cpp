
#include <gtest/gtest.h>

#include "treeinterp/BTFactory.h"

TEST(BT, JsonEditor) {
    std::string testProject = "sample";
    std::string testTree = "sAmPleNamE";
    JsonReader pathReader;

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
