//
// Created by baris on 15/11/18.
//
#include "BTImport.h"
#include "../skills/GoToPos.h"
#include "../skills/Kick.h"
#include "../bt/Node.hpp"
#include "../bt/tactics/DemoTactic.h"
#include "../bt/tactics/ParallelSequenceTest.h"
#include "../bt/tactics/VictoryDanceTactic.h"
#include "../skills/Rotate.h"

#ifndef ROBOTEAM_AI_SWITCHES_H
#define ROBOTEAM_AI_SWITCHES_H

class Switches {

    public:

        static std::vector<std::string> tacticJsonFileNames;

        static std::vector<std::string> strategyJsonFileNames;

        static bt::Node::Ptr nonLeafSwitch(std::string name);

        static bt::Node::Ptr leafSwitch(std::string name, bt::Blackboard::Ptr properties);

        static bt::Node::Ptr tacticSwitch(std::string name, bt::Blackboard::Ptr properties);


};

#endif //ROBOTEAM_AI_SWITCHES_H
