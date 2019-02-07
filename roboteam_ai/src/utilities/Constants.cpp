//
// Created by mrlukasbos on 7-2-19.
//

#include "Constants.h"

namespace rtt {
namespace ai {

// static variable initialization
bool Constants::useGrSim = false;
std::map<std::string, double> Constants::doubles;
std::map<std::string, bool> Constants::bools;
std::map<std::string, int> Constants::integers;
std::map<std::string, QColor> Constants::colors;
Constants::_init Constants::_initializer;

double Constants::getDouble(const std::string &name) {
    if (doubles.find(name) != doubles.end() ) {
        return doubles[name];
    }
    ROS_ERROR("Double constant not found: %s", name.c_str());
    return 0.0;
}

int Constants::getInt(const std::string &name) {
    if (integers.find(name) != integers.end() ) {
        return integers[name];
    }
    ROS_ERROR("Integer constant not found: %s", name.c_str());
    return 0;
}

bool Constants::getBool(const std::string &name) {
    if (bools.find(name) != bools.end() ) {
        return bools[name];
    }
    ROS_ERROR("Bool constant not found: %s", name.c_str());
    return false;
}

QColor Constants::getColor(const std::string &name) {
    if (colors.find(name) != colors.end() ) {
        return colors[name];
    }
    ROS_ERROR("Double constant not found: %s", name.c_str());
    return { 0, 0, 0, 0 }; // transparent
}

} // ai
} // rtt
