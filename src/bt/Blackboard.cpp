#include <utility>

#include <string>
#include <memory>
#include "roboteam_proto/StringEntry.pb.h"
#include "bt/Blackboard.hpp"

namespace bt {

void Blackboard::setBool(std::string key, bool value) {
    bools[key] = value;
}

bool Blackboard::getBool(std::string key) {
    // if (bools.find(key) == bools.end()) {
    //     bools[key] = false;
    // }
    // bools[key] deafult constructs if no exist
    // https://en.cppreference.com/w/cpp/container/map/operator_at
    // If an insertion is performed, the mapped value is value-initialized (default-constructed for class types, zero-initialized otherwise) and a reference to it is returned.

    // https://godbolt.org/z/87GUTV
    return bools[key];
}

bool Blackboard::hasBool(std::string key) const {
    return bools.find(key) != bools.end();
}

void Blackboard::setInt(std::string key, int value) {
    ints[key] = value;
}

int Blackboard::getInt(std::string key) {
    if (ints.find(key) == ints.end()) {
        ints[key] = - 1;
    }
    return ints[key];
}

bool Blackboard::hasInt(std::string key) const {
    return ints.find(key) != ints.end();
}

void Blackboard::setFloat(std::string key, float value) {
    floats[key] = value;
}

float Blackboard::getFloat(std::string key) {
    // same story as line 15
    return floats[key];
}

bool Blackboard::hasFloat(std::string key) const {
    return floats.find(key) != floats.end();
}

void Blackboard::setDouble(std::string key, double value) {
    doubles[key] = value;
}

double Blackboard::getDouble(std::string key) {
    return doubles[key];
}

bool Blackboard::hasDouble(std::string key) const {
    return doubles.find(key) != doubles.end();
}

void Blackboard::setString(std::string key, std::string value) {
    strings[key] = std::move(value);
}

std::string Blackboard::getString(std::string key) {
    return strings[key];
}

bool Blackboard::hasString(std::string key) const {
    return strings.find(key) != strings.end();
}

rtt::Vector2 Blackboard::getVector2(std::string key) {
    return vectors[key];
}
void Blackboard::setVector2(std::string key, rtt::Vector2 value) {
    vectors[key] = value;

}
bool Blackboard::hasVector2(std::string key) const {
    return vectors.find(key) != vectors.end();
}

} // bt
