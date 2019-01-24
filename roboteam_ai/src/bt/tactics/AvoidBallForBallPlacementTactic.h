//
// Created by mrlukasbos on 24-1-19.
//

#ifndef ROBOTEAM_AI_AVOIDBALLFORBALLPLACEMENTTACTIC_H
#define ROBOTEAM_AI_AVOIDBALLFORBALLPLACEMENTTACTIC_H

#include "../Tactic.h"

namespace bt {

class AvoidBallForBallPlacementTactic : public Tactic {
public:
    std::map<std::string, robotType> robots;
    explicit AvoidBallForBallPlacementTactic(std::string name, Blackboard::Ptr blackboard);
    void initialize() override;
};
}

#endif //ROBOTEAM_AI_AVOIDBALLFORBALLPLACEMENTTACTIC_H
