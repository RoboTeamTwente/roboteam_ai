//
// Created by baris on 29-11-18.
//

#ifndef ROBOTEAM_AI_DEFAULTTACTIC_H
#define ROBOTEAM_AI_DEFAULTTACTIC_H

#include <mutex>
#include "../Tactic.h"
#include <roboteam_ai/src/analysis/DecisionMaker.h>


namespace bt {

class DefaultTactic : public Tactic {
    FRIEND_TEST(DefaultTacticTest, default_general_tactic_works);
    FRIEND_TEST(DefaultTacticTest, offensive_defensive_midfield_tactics_work);
    private:
        int amountToTick = -1;
        void claimRobots(int amount);
        void disClaimRobots(int amount);
        bool updateRobots();
        std::pair<std::string, RobotType> getNextClaim();
        std::pair<std::string, RobotType> getLastClaim();
        void parseType(const std::string& typee);
        void updateStyle();
        rtt::ai::analysis::DecisionMaker maker;
        void convert(const std::vector<std::pair<std::string, RobotType>>& unit);

    public:
        std::vector<std::tuple<int, std::string, RobotType>> robots;
        DefaultTactic(std::string name, Blackboard::Ptr blackboard, const std::vector<std::pair<std::string, RobotType>>& robots);
        void initialize() override;
        Node::Status update() override;
        enum TacticType : short {
          Defensive,
          Middle,
          Offensive,
          General
        };
        TacticType thisType = TacticType::General;

};
}

#endif //ROBOTEAM_AI_DEFAULTTACTIC_H
