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

    for ( auto &property : jsonLeaf["properties"].get<json::object_t>() ) {
        std::vector<double> vec;
        int varType = checkVarTypeOfString(property.first, jsonLeaf, vec);
        switch (varType) {
        case 0:BB->setBool(property.first, false);
            break;
        case 1:BB->setBool(property.first, true);
            break;
        case 2:BB->setInt(property.first, (int) round(vec[0]));
            break;
        case 3:BB->setDouble(property.first, vec[0]);
            break;
        case 4: {
            if (vec.size() == 2) {
                rtt::Vector2 vec2 = {vec[0], vec[1]};
                BB->setVector2(property.first, vec2);
            } else if (vec.size() == 1) {
                BB->setString(property.first, property.second);
            } else {
                //TODO: add vector3 or vectorN, with more than 3 elements here.
                BB->setString(property.first, property.second);
            }
        break;
        }
        case -1:
        default:BB->setString(property.first, property.second);
        }
    }
    return BB;
}

int PropertiesParser::checkVarTypeOfString(std::string keyName, json jsonLeaf, std::vector<double> &vec) {

    std::string strKey = (std::string)jsonLeaf["properties"][keyName];
    unsigned long size = strKey.size();
    int it = 0;
    int varType;

    // Convert to individual characters
    char charKey[100];
    for(int i=0; i<strKey.size(); i++){
        charKey[i]= strKey[i];
    }
    if (strKey == "true") return 1;
    else if (strKey == "false") return 0;
    else if (charKey[0] == vectorStartChar && charKey[size-1] == vectorEndChar && (std::isdigit(strKey[1]) || charKey[1] == dot)) {
        // we are dealing with a vector
        double number;

        varType = getNumberFromString(strKey, charKey, ++it, number);
        if (varType == -1) return varType;
        vec.push_back(number);
        it++;
        while (charKey[it] ==  comma || charKey[it] == space) {
            while (charKey[it] == space) it++;
            varType = getNumberFromString(strKey, charKey, it, number);
            if (varType == -1) return varType;
            vec.push_back(number);
            it++;
        }
        if (charKey[it] != vectorEndChar) {
            varType = - 1;
            return varType;
        }
        varType = 4;
        return varType;
    } else {
        double number;
        varType = getNumberFromString(strKey, charKey, it, number);
        if (varType == -1) return varType;
        vec.push_back(number);
        return varType;
    }
}


/// get the next value from the string starting at character number 'it'.
/// return -1 = error, 2 = int, 3 = double
int PropertiesParser::getNumberFromString(std::string strKey, char charKey[100], int &it, double &number) {
    double sum = 0;
    if (std::isdigit(strKey[it])) {
        while (std::isdigit(strKey[it]) && it < strKey.size()) {

            sum = 10*sum + (int) (charKey[it] - '0');
            it ++;
        }
        if (charKey[it] == dot && it < strKey.size()) {
            double multiplier = 1.0;
            it ++;
            if (std::isdigit(strKey[it])) {
                while (std::isdigit(strKey[it])) {
                    multiplier *= 0.1;
                    sum += (double) (charKey[it] - '0')*multiplier;
                    it ++;
                }
                number = sum;
                return 1; // number with a dot -> double
            } else {
                return -1; // no digit after the dot
            }
        } else return 0; // number without a dot -> int

    } else return -1; // not starting with a digit
}


