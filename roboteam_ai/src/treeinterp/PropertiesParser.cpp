//
// Created by baris on 31/10/18.
//

#include "PropertiesParser.h"

bt::Blackboard::Ptr PropertiesParser::parse(PropertiesParser::json jsonLeaf) {

    bt::Blackboard::Ptr BB = std::make_shared<bt::Blackboard>();

    if (! jsonReader.checkIfKeyExists("properties", jsonLeaf)) {
        return BB;
    }

    for (auto property = jsonLeaf["properties"].begin(); property != jsonLeaf["properties"].end(); ++ property) {

        std::vector<double> vec;
        PropertiesParser::type varType = checkVarTypeOfString(property.key(), jsonLeaf, vec);
        switch (varType) {
            case Bool_False:
                BB->setBool(property.key(), false);
                break;
            case Bool_True:
                BB->setBool(property.key(), true);
                break;
            case Int:
                BB->setInt(property.key(), (int) round(vec[0]));
                break;
            case Double:
                BB->setDouble(property.key(), vec[0]);
                break;
            case Vector: {
                if (vec.size() == 2) {
                    rtt::Vector2 vec2 = {vec[0], vec[1]};
                    BB->setVector2(property.key(), vec2);
                }
                else if (vec.size() == 1) {
                    BB->setString(property.key(), property.value());
                }
                else {
                    //TODO: add vector3 or vectorN, with more than 3 elements here.
                    BB->setString(property.key(), property.value());
                }
                break;
            }
            default:
                BB->setString(property.key(), property.value());
        }
    }
    return BB;
}

PropertiesParser::type PropertiesParser::checkVarTypeOfString(std::string keyName, json jsonLeaf,
        std::vector<double> &vec) {

    std::string strKey = (std::string) jsonLeaf["properties"][keyName];
    if (strKey.empty()) {
        ROS_ERROR("PARSING EMPTY STRING!! this could give errors");
        return String;
    }
    int it = 0;
    type varType;

    // Convert to individual characters
    std::vector<char> charKey;
    for (char i : strKey) {
        charKey.push_back(i);
    }

    if (strKey == "true") return Bool_True;
    else if (strKey == "false") return Bool_False;
    else if (charKey.front() == vectorStartChar && charKey.back() == vectorEndChar) {
        // we are dealing with a vector
        it ++;
        while (charKey[it] == space)
            it ++;                  // skip spaces and move to the first character after "{" in the string
        if (! (std::isdigit(strKey[it]) || (charKey[it] == minus)))
            return String;       // check if the first character is a digit (0, 1, ... , 9)

        double number;                                      // get the variable type and the value of the next unit in the vector
        varType = getNumberFromString(strKey, charKey, it, number);
        if (varType == String) return String;
        vec.push_back(number);
        while (charKey[it] == space) it ++;
        while (charKey[it] == comma) {
            it ++;
            while (charKey[it] == space) it ++;
            varType = getNumberFromString(strKey, charKey, it, number);
            if (varType == String) return String;
            vec.push_back(number);
        }
        // last check to see if we are at the end of the string
        while (charKey[it] == space) it ++;
        if (charKey[it] != vectorEndChar || it != (signed) charKey.size() - 1) return String;
        else return Vector;
    }
    else {
        double number;
        varType = getNumberFromString(strKey, charKey, it, number);
        vec.push_back(number);
        return varType;
    }
}

/// get the next value from the string starting at character number 'it'.
PropertiesParser::type PropertiesParser::getNumberFromString(std::string strKey, std::vector<char> charKey, int &it,
        double &number) {
    double sum = 0;
    bool negNum = false;
    if (std::isdigit(strKey[it]) || charKey[it] == minus) {
        if (charKey[it] == minus) {
            while (charKey[++ it] == space);
            negNum = true;
        }
        while (std::isdigit(strKey[it]) && it < (signed) strKey.size()) {

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
                if (negNum) number = - sum;
                else number = sum;
                return Double; // number with a dot -> double
            }
            else {
                return String; // no digit after the dot
            }
        }
        else {
            if (negNum) number = - sum;
            else number = sum;
            return Int; // number without a dot -> int
        }

    }
    else return String; // not starting with a digit
}


