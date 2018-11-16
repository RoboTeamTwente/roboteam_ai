//
// Created by baris on 15/11/18.
//
#include "BTImport.h"
#include "../skills/GoToPos.h"
#include "../skills/Kick.h"

#ifndef ROBOTEAM_AI_SWITCHES_H
#define ROBOTEAM_AI_SWITCHES_H

class Switches {

    public:

        static std::vector<std::string> tacticNames;

        static std::vector<std::string> strategyNames;

        static bt::Node::Ptr nonLeafSwitch(std::string name);

        static bt::Node::Ptr leafSwitch(std::string name, bt::Blackboard::Ptr properties);

        static bt::Node::Ptr tacticSwitch(std::string name, bt::Blackboard::Ptr properties);


};

#endif //ROBOTEAM_AI_SWITCHES_H
