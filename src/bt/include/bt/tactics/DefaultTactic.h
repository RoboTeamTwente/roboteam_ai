//
// Created by baris on 29-11-18.
//

#ifndef ROBOTEAM_AI_DEFAULTTACTIC_H
#define ROBOTEAM_AI_DEFAULTTACTIC_H

#include <analysis/DecisionMaker.h>
#include <utilities/RobotDealer.h>
#include <src/bt/include/bt/Node.hpp>
#include <mutex>

namespace bt {

class DefaultTactic : public Node {
    FRIEND_TEST(DefaultTacticTest, default_general_tactic_works);
    FRIEND_TEST(DefaultTacticTest, offensive_defensive_midfield_tactics_work);
    FRIEND_TEST(DefaultTacticTest, claim_test);

    using RobotType = rtt::ai::robotDealer::RobotType;

   private:
    int amountToTick = -1;
    void claimRobots(int amount);
    void disClaimRobots(int amount);
    bool updateRobots();
    std::pair<std::string, RobotType> getNextClaim();
    std::pair<std::string, RobotType> getLastClaim();
    void parseType(const std::string &typee);
    void updateStyle();
    rtt::ai::analysis::DecisionMaker maker;
    void convert(const std::vector<std::pair<std::string, RobotType>> &unit);
    std::set<int> robotIDs = {};
    int claimedRobots = 0;
    std::string name;
    std::vector<Node::Ptr> children;
    void giveProperty(std::string a, std::string b) override;
    std::vector<Node::Ptr> getChildren() override;

   public:
    std::vector<std::tuple<int, std::string, RobotType>> robots;
    DefaultTactic(std::string name, Blackboard::Ptr blackboard, const std::vector<std::pair<std::string, RobotType>> &robots);

    void initialize() override;
    Node::Status update() override;
    enum TacticType : short { Defensive, Middle, Offensive, General };
    TacticType thisType = TacticType::General;
    void terminate(Node::Status s) override;
    std::string node_name() override;
    void addChild(Node::Ptr newChild) override;
};
}  // namespace bt

#endif  // ROBOTEAM_AI_DEFAULTTACTIC_H
