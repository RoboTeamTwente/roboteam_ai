//
// Created by baris on 15/11/18.
//
#include "BTImport.h"
#include "../bt/Node.hpp"


//  ______________________
//  |                    |
//  |   INCLUDE TACTICS  |
//  |____________________|
//

#include "../bt/tactics/DemoTactic.h"
#include "../bt/tactics/ParallelSequenceTest.h"
#include "../bt/tactics/VictoryDanceTactic.h"
#include "../bt/tactics/RandomTactic.h"
#include "../bt/tactics/DefaultTactic.h"
#include "../bt/tactics/HaltTactic.h"

//  ______________________
//  |                    |
//  |   INCLUDE SKILLS   |
//  |____________________|
//

#include "../skills/Chip.h"
#include "../skills/Dribble.h"
#include "../skills/GoToPosLuTh.h"
#include "../skills/GoToPosLuTh_OLD.h"
#include "../skills/Halt.h"
#include "../skills/Kick.h"
#include "../skills/Rotate.h"
#include "../skills/RotateToAngle.h"
#include "../skills/GoToPos.h"
#include "../skills/Keeper.h"

//  ______________________
//  |                    |
//  | INCLUDE CONDITIONS |
//  |____________________|
//

#include "../conditions/HasBall.hpp"


#ifndef ROBOTEAM_AI_SWITCHES_H
#define ROBOTEAM_AI_SWITCHES_H

class Switches {
        using robotType = robotDealer::RobotDealer::RobotType;

    public:

        static std::vector<std::string> tacticJsonFileNames;

        static std::vector<std::string> strategyJsonFileNames;

        static std::vector<std::string> keeperJsonFiles;

        static bt::Node::Ptr nonLeafSwitch(std::string name);

        static bt::Node::Ptr leafSwitch(std::string name, bt::Blackboard::Ptr properties);

        static bt::Node::Ptr tacticSwitch(std::string name, bt::Blackboard::Ptr properties);


};

#endif //ROBOTEAM_AI_SWITCHES_H
