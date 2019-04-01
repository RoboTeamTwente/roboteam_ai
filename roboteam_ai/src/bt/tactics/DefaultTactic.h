//
// Created by baris on 29-11-18.
//

#ifndef ROBOTEAM_AI_DEFAULTTACTIC_H
#define ROBOTEAM_AI_DEFAULTTACTIC_H

#include <mutex>
#include "../Tactic.h"

namespace bt {

class DefaultTactic : public Tactic {
    private:
        std::mutex amountMutex;
        int previousAmount = -1;
        int amountToTick = -1;
        void claimRobots();
        void disClaimRobots();
        bool updateRobots();
        int claimIndex = 0;
        std::pair<std::string, robotType> getNextClaim();
        std::pair<std::string, robotType> getLastClaim();




    public:
        int robotsNeeded = -1;
        std::map<std::string, robotType> robots;
        DefaultTactic(std::string name, Blackboard::Ptr blackboard, std::map<std::string, robotType> robots);
        void initialize() override;
        Node::Status update() override;
        void setRoleAmount(int amount);

};
}

#endif //ROBOTEAM_AI_DEFAULTTACTIC_H
