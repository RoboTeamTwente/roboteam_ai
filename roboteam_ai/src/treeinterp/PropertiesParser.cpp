//
// Created by baris on 31/10/18.
//

#include <utility>
#include "PropertiesParser.h"
#include "../bt/Blackboard.hpp"
#include <math.h>
#include <string>
#include <vector>

bt::Blackboard::Ptr PropertiesParser::parse(PropertiesParser::json jsonLeaf) {
    bt::Blackboard::Ptr BB = std::make_shared<bt::Blackboard>();

    for (auto &property : jsonLeaf["properties"].get<json::object_t>()) {

        std::vector<double> vec;
        PropertiesParser::type varType = checkVarTypeOfString(property.first, jsonLeaf, vec);
        switch (varType) {
        case Bool_False:
            BB->setBool(property.first, false);
            break;
        case Bool_True:
            BB->setBool(property.first, true);
            break;
        case Int:
            BB->setInt(property.first, (int) round(vec[0]));
            break;
        case Double:
            BB->setDouble(property.first, vec[0]);
            break;
        case Vector: 
            {
            if (vec.size() == 2) {
                rtt::Vector2 vec2 = {vec[0], vec[1]};
                BB->setVector2(property.first, vec2);
            }
            else if (vec.size() == 1) {
                BB->setString(property.first, property.second);
            }
            else {
                //TODO: add vector3 or vectorN, with more than 3 elements here.
                BB->setString(property.first, property.second);
            }
            break;
        }
        default:
            BB->setString(property.first, property.second);
        }
    }
    return BB;
}

PropertiesParser::type PropertiesParser::checkVarTypeOfString(std::string keyName, json jsonLeaf,
        std::vector<double> &vec) {

    std::string strKey = (std::string) jsonLeaf["properties"][keyName];
    unsigned long size = strKey.size();
    int it = 0;
    type varType;

    // Convert to individual characters
    char charKey[100];
    for (int i = 0; i < strKey.size(); i ++) {
        charKey[i] = strKey[i];
    }
    if (strKey == "true") return Bool_True;
    else if (strKey == "false") return Bool_False;
    else if (charKey[0] == vectorStartChar && charKey[size - 1] == vectorEndChar) {
        // we are dealing with a vector

        while (charKey[it] == space) it++;                  // skip spaces and move to the first character after "{" in the string
        if (!std::isdigit(strKey[it])) return String;       // check if the first character is a digit (0, 1, ... , 9)

        double number;                                      // get the variable type and the value of the next unit in the vector
        varType = getNumberFromString(strKey, charKey, ++ it, number);
        if (varType == String) return String;
        vec.push_back(number);
        it ++;
        while (charKey[it] == comma || charKey[it] == space) {
            it ++;
            while (charKey[it] == space) it ++;
            varType = getNumberFromString(strKey, charKey, it, number);
            if (varType == String) return String;
            vec.push_back(number);
            it ++;
        }
        // last check to see if we are at the end of the string
        if (charKey[it] != vectorEndChar && it != size - 1) return String;
        else return Vector;
    }
    else {
        double number;
        varType = getNumberFromString(strKey, charKey, it, number);
        return varType;
    }
}

/// get the next value from the string starting at character number 'it'.
PropertiesParser::type PropertiesParser::getNumberFromString(std::string strKey, char charKey[100], int &it, double &number) {
    double sum = 0;
    if (std::isdigit(strKey[it])) {
        while (std::isdigit(strKey[it]) && it < strKey.size()) {

            sum = 10*sum + (int) (charKey[it] - '0');
            it ++;
        }
        if (charKey[it] == dot) {
            double multiplier = 1.0;
            if (std::isdigit(strKey[++ it])) {
                while (std::isdigit(strKey[it])) {
                    multiplier *= 0.1;
                    sum += (double) (charKey[it] - '0')*multiplier;
                    it ++;
                }
                number = sum;
                return Double; // number with a dot -> double
            }
            else {
                return String; // no digit after the dot
            }
        }
        else return Int; // number without a dot -> int

    }
    else return String; // not starting with a digit
}


