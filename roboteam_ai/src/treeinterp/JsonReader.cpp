//
// Created by baris on 09/10/18.
//

#include "JsonReader.h"

#define GetCurrentDir getcwd // Needed for the path finding

/// Returns the file path from /home/ to the given json name
std::string JsonReader::getFilePath(std::string name) {

    char cCurrentPath[FILENAME_MAX];
    if (! GetCurrentDir(cCurrentPath, sizeof(cCurrentPath))) {
        std::cerr << "gh" << std::endl;
    }

    // Get the string version
    std::string fullPath = cCurrentPath;
    auto splitTed = JsonReader::split(fullPath, '/');
    std::string smallPath;
    for (const auto &word : splitTed) {
        if (smallPath.find("roboteam_ai") != std::string::npos) {
            break;
        }
        smallPath.append(word + "/");
    }
    // should be at /home/[user]/roboteamtwente/workspace/src/roboteam_ai/ ish right now
    smallPath.append("roboteam_ai/src/jsons/" + name + ".json");
    return smallPath;
}

/// Splits a string with the given char into a vector
std::vector<std::string> JsonReader::split(std::string s, char c) {
    std::vector<std::string> v;
    std::string::size_type i = 0;
    std::string::size_type j = s.find(c);

    while (j != std::string::npos) {
        v.push_back(s.substr(i, j - i));
        i = ++ j;
        j = s.find(c, j);

        if (j == std::string::npos)
            v.push_back(s.substr(i, s.length()));
    }
    return v;
}

/// Returns JSON object from a file name
json JsonReader::readJSON(std::string fileName) {
    std::string filePath = JsonReader::getFilePath(std::move(fileName));
    std::ifstream ifs(filePath);
    json bigJSON = json::parse(ifs);
    return bigJSON;
}

/// Checks if a key exists in a json object
bool JsonReader::checkIfKeyExists(std::string key, json json) {
    return (json.find(key) != json.end());
}

void JsonReader::editJSON(std::string fileName, std::string treeID, std::string field, std::string newValue) {
    // read json file
    json fileJson = readJSON(fileName);
    // edit json file
    for (json &tree :fileJson["data"]["trees"]) {
        if (tree["id"] == treeID) {
            tree[field] = newValue;
            break;
        }
    }
    //write it back to the same place
    std::ofstream ofs(JsonReader::getFilePath(fileName));
    ofs << fileJson;
    ofs.close();
}