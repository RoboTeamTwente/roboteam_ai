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
        std::pair<std::string, RobotType> getNextClaim();
        std::pair<std::string, RobotType> getLastClaim();
        void parseType(std::string typee);





    public:
        int robotsNeeded = -1;
        std::map<std::string, RobotType> robots;
        DefaultTactic(std::string name, Blackboard::Ptr blackboard, std::map<std::string, RobotType> robots);
        void initialize() override;
        Node::Status update() override;
        void setRoleAmount(int amount);
        enum TacticType : short {
          Defensive,
          Middle,
          Offensive,
          General
        };
        TacticType thisType;

};
}

#endif //ROBOTEAM_AI_DEFAULTTACTIC_H
