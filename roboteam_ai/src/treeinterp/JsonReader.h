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

    FRIEND_TEST(JsonBasics, JsonTest);

    std::string getFilePath(std::string name);

    std::vector<std::string> split(std::string s, char c);

protected:

public:

    json readJSON(std::string fileName);

    JsonReader() = default;

    void printJson(const json& j) {
        std::string dump = j.dump();
        std::cout << "JSON print out:\n: " + dump << std::endl;
    }

    bool checkIfKeyExists(std::string key, json json);


};

#endif //ROBOTEAM_AI_JSONREADER_H
