//
// Created by jesse on 14.10.19.
//
#ifndef RTT_TREE_PROTO_TYPE_H
#define RTT_TREE_PROTO_TYPE_H

#include "treeinterp/tactics/DefaultTactic.h"
#include "bt/BehaviorTree.h"
#include "bt/Role.h"

namespace bt {

class OffensiveStrategy {
   public:
    /**
     * Constructor that initializes the robots vector
     */
    OffensiveStrategy();
    /**
     * Creates a Play behaviour tree. This tree is created in BTFactory. The tree that is created is also given to the OffensiveStrategy object, so it can be traced and adjusted.
     * @return the behaviour tree that contains this strategy
     */
    std::shared_ptr<bt::BehaviorTree> createOffensiveStrategy();

   private:
    /**
     * Vector that is used by RobotDealer to determine how to match robots to robot IDs.
     */
    std::vector<std::pair<std::string, rtt::ai::robotDealer::RobotType>> robots;

    /**
     * The tree that is given to the offensive strategy
     */
    std::shared_ptr<bt::BehaviorTree> tree;

    /**
     * Creates a defensive tactic. This a tree, which is an ordered collection of nodes with childen (and parents). This is then put into the strategy
     * created in createNormalPlayStrategy(). In this function, normally, the tactic is given 11 children (11 = number of robots).
     * @param bb, the blackboard given to the tactic (that saves what the TacticType is)
     * @return a defensive tacic that
     */
    std::shared_ptr<bt::DefaultTactic> createOffensiveTactic(std::shared_ptr<Blackboard> bb);

    /**
     * Create an offender robot with a certain name. The role needs to be given to the tactic (so the tactic has 8 children that are roles)
     * @param name, the name of the robot
     * @return an offender role, a subtree containing the nodes necessary to execute this role.
     */
    std::shared_ptr<bt::Role> createOffenderRole(std::string name);
};
}  // namespace bt

#endif  // RTT_TREE_PROTO_TYPE_H
