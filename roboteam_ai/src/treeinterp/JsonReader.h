//
// Created by baris on 09/10/18.
//

#ifndef ROBOTEAM_AI_JSONREADER_H
#define ROBOTEAM_AI_JSONREADER_H

#include "json.h"
#include "../bt/BehaviorTree.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <gtest/gtest_prod.h>
#include "vector"
#include "../bt/composites/MemSequence.hpp"
#include "../bt/Leaf.hpp"
#include <map>
#include <unistd.h>

using json = nlohmann::json;

class JsonReader {

private:

    FRIEND_TEST(Tree, JsonTest);

    std::string getFilePath(std::string name);

    std::vector<std::string> split(std::string s, char c);

protected:

public:

    json readJSON(std::string fileName);

    JsonReader() = default;

};

#endif //ROBOTEAM_AI_JSONREADER_H
