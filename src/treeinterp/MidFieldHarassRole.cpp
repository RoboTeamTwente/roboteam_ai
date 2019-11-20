//
// Created by jesse on 19.10.19.
//

#include <include/roboteam_ai/TreeHelper/BeingPassedToHelper.h>
#include <include/roboteam_ai/bt/composites/Selector.hpp>
#include "bt/RoleDivider.h"
#include "bt/BehaviorTree.hpp"
#include "bt/decorators/Repeater.hpp"
#include "treeinterp/TreeProtoType.h"
#include "bt/tactics/DefaultTactic.h"
#include "bt/Role.h"
#include "skills/gotopos/GoToPos.h"
#include "skills/Attack.h"
#include "include/roboteam_ai/treeinterp/MidFieldHarassRole.h"

namespace bt {


    std::shared_ptr<Role> MidFieldHarassRole::createMidFieldHarassRole(std::string name) {
        /// Create the role tree here
        std::shared_ptr<bt::Role> roleNode = std::make_shared<bt::Role>("meow");
        bt::BeingPassedToHelper helper = bt::BeingPassedToHelper();
        auto subNode = helper.createBeingPassedToChecker();

        /// TOP LAYER:
        std::shared_ptr<bt::Selector> select = std::make_shared<bt::Selector>();
        select->addChild(subNode);
        auto localbb = std::make_shared<bt::Blackboard>();
        localbb->setString("ROLE", "a");
        std::shared_ptr<rtt::ai::Attack> attack = std::make_shared<rtt::ai::Attack>("attack",  localbb);
        select->addChild(attack);
        // did some bad things, for testing purposes only!

        roleNode->addChild(select);
        roleNode->setRole("r1");

        return roleNode;
    }

    MidFieldHarassRole::MidFieldHarassRole() {

    }


}
