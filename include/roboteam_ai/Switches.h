//
// Created by baris on 15/11/18.
//
#ifndef ROBOTEAM_AI_SWITCHES_H
#define ROBOTEAM_AI_SWITCHES_H

#include <iostream>
#include <utilities/RobotDealer.h>
#include <bt/Blackboard.hpp>
#include <bt/Node.hpp>

class Switches {
        private:
    /**
     * Useless std::map copy right here
     */
    static void runErrorHandler(std::map<std::string, std::map<std::string, rtt::ai::robotDealer::RobotType>> tactics);

    public:

        static std::vector<std::string> tacticJsonFileNames;

        static std::vector<std::string> strategyJsonFileNames;

        static std::vector<std::string> keeperJsonFiles;

        static bt::Node::Ptr nonLeafSwitch(std::string name);

        static bt::Node::Ptr leafSwitch(std::string name, bt::Blackboard::Ptr properties);

        static bt::Node::Ptr tacticSwitch(std::string name, bt::Blackboard::Ptr properties);
};

#endif //ROBOTEAM_AI_SWITCHES_H
