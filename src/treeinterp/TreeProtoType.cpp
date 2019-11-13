//
// Created by jesse on 14.10.19.
//

#include "bt/RoleDivider.h"
#include "bt/BehaviorTree.hpp"
#include "bt/decorators/Repeater.hpp"
#include "treeinterp/TreeProtoType.h"
#include "bt/tactics/DefaultTactic.h"
#include "bt/Role.h"
#include "skills/gotopos/GoToPos.h"
#include "skills/Attack.h"

namespace bt {

    /**
     * In this constructor (which is always called by default when initializing the
     */
    TreeProtoType::TreeProtoType(){
        // Set the robottypes for the robot so the robotdealer can decide which robot should do what
        this->robots = {
                {"o1", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o2", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o3", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o4", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o5", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o6", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o7", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o8", rtt::ai::robotDealer::RobotType::RANDOM}
        };
    }


/**
 * Creates offensive strategy behaviour tree
 * @return the behaviour tree corresponding to offensive strategy
 */
std::shared_ptr<BehaviorTree> TreeProtoType::createOffensiveStrategy() {
    std::shared_ptr<Blackboard> bb = std::make_shared<Blackboard>();
    // Set the tactictype so the robotdealer can divide the robots
    // Currently there are 2 options: General and (Offensive/Defensive/Midfield).
    // This property is given to the blackboard, which is then given to the tactic. As far as @RobotJesse could tell,
    // this is only used in the tactic itself to determine which robots it should tick
    // in the updateStyle function of DefaultTactic.
    bb->setString("TacticType", "General");

    std::shared_ptr<DefaultTactic> offensiveTactic = createOffensiveTactic(bb);
    offensiveTactic->setProperties(bb);

    auto tree = std::make_shared<BehaviorTree>("defendertree");
    tree->SetRoot(offensiveTactic);

    return tree;
}

/**
 * Creates offensive tactic for as many robots as are in the robots vector.
 * @param bb the blackboard that needs to be given to the offensive tactic. The blackboard
 * needs to have: bb->setString("TacticType", "General"), where General can be replaced with Offensive, Defensive, etc
 * @return offensive tactic with as many attacker roles as the size of the robots vector.

 */
std::shared_ptr<DefaultTactic> TreeProtoType::createOffensiveTactic(std::shared_ptr<Blackboard> bb) {
    // create a default tactic which will be used to build the offensive tactic
    std::shared_ptr<DefaultTactic> offensiveTactic = std::make_shared<DefaultTactic>("offensiveTactic", bb, robots);

    // Creating the roles for all the robots in the tactic:
    for (int i = 1; i < robots.size(); i++) {
        std::string name = "o" + std::to_string(i);
        auto temp = createOffenderRole(name);
        std::shared_ptr<Role> temprole = createOffenderRole(name);
        temp->giveProperty("ROLE", name);
        offensiveTactic->addChild(temprole);
    }

    return offensiveTactic;
}

/**
 *
   This function creates an offender role. This role consists of a repeater with as child an attack skill.
   It is important to remember that the order in which the nodes are added is important.
 * @param name the name that the role should have
 * @return the tree corresponding to an offender role
 */
std::shared_ptr<Role> TreeProtoType::createOffenderRole(std::string name) {
    // set the rolename for the current role. This is important because the robotdealer decides how to deal the robots based on their rolenames
    // The property "ROLE" should be set to the name of the role, which should correspond to the rolename
    // found in the robotdealer robots vector
    auto localbb = std::make_shared<Blackboard>();
    localbb->setString("ROLE", name);
    std::shared_ptr<Role> role = std::make_shared<Role>(name);

    // Give the role the blackboard. This is used by the robotdealer to find the robot
    // TODO: figure out the exact difference between giving ROLE property to Role blackboard vs giving ROLE property to Skill blackboard
    // It seems ROLE just needs to be given to both. This is always safe.
    role->setProperties(localbb);

    // Make a repeater with the child "attack"
    std::shared_ptr<Repeater> repeater = std::make_shared<Repeater>();
    std::shared_ptr<rtt::ai::Attack> attack = std::make_shared<rtt::ai::Attack>("attack",  localbb);

    repeater->addChild(attack);
    role->addChild(repeater);
    return role;

}



}
