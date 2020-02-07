//
// Created by jessevw on 03.12.19.
//

#include "include/roboteam_ai/treeinterp/PassRole.h"

#include <include/roboteam_ai/skills/Halt.h>
#include <include/roboteam_ai/skills/Pass.h>

#include <bt/composites/MemSequence.h>

#include "bt/BehaviorTree.h"
#include "bt/Role.h"
#include "bt/decorators/Repeater.h"
#include "skills/Attack.h"
#include "skills/gotopos/GoToPos.h"
#include "treeinterp/OffensiveStrategy.h"
#include "treeinterp/tactics/DefaultTactic.h"

namespace bt {

/**
 * Creates pass role. This role passes if there is a pass available, otherwise it halts
 * @param rolename the name of the role (must correspond to the rolename in the robots vector of the tactic
 * @return role for passing and then halting
 */
std::shared_ptr<Role> PassRole::createPassRole(std::string rolename) {
    auto localbb = std::make_shared<bt::Blackboard>();
    std::shared_ptr<Role> roleNode = std::make_shared<Role>(rolename);
    auto passSkill = std::make_shared<rtt::ai::Pass>("pass", localbb);
    auto repeater = std::make_shared<bt::Repeater>();
    auto halt = std::make_shared<rtt::ai::Halt>("halt", localbb);

    auto memSeq = std::make_shared<bt::MemSequence>();

    memSeq->addChild(passSkill);
    memSeq->addChild(repeater);
    repeater->addChild(halt);
    roleNode->addChild(memSeq);
    roleNode->setRoleString(rolename);

    return roleNode;
}

}  // namespace bt
