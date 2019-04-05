//
// Created by baris on 29-11-18.
//

#ifndef ROBOTEAM_AI_DEFAULTTACTIC_H
#define ROBOTEAM_AI_DEFAULTTACTIC_H

#include "../Tactic.h"

namespace bt {

class DefaultTactic : public Tactic {
    public:
        int robotsNeeded = -1;
        std::map<std::string, RobotType> robots;
        DefaultTactic(std::string name, Blackboard::Ptr blackboard, std::map<std::string, RobotType> robots);
        void initialize() override;
        Node::Status update() override;
        void claimRobots();
};
}

#endif //ROBOTEAM_AI_DEFAULTTACTIC_H
