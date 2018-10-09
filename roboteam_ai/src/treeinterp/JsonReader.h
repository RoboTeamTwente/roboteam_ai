//
// Created by baris on 09/10/18.
//

#ifndef ROBOTEAM_AI_JSONREADER_H
#define ROBOTEAM_AI_JSONREADER_H

#include "json.h"
#include <iostream>
#include <fstream>
#include <string>
#include <gtest/gtest_prod.h>
#include "vector"
#include <map>
#include <unistd.h>
#include "BTImport.h"

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
