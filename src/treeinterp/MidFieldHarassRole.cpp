//
// Created by jesse on 19.10.19.
//

#include <include/roboteam_ai/TreeHelper/BeingPassedToHelper.h>
#include <include/roboteam_ai/bt/composites/Selector.hpp>
#include <include/roboteam_ai/TreeHelper/RobotOutOfFieldHelper.h>
#include "bt/BehaviorTree.hpp"
#include "bt/Role.h"
#include "skills/gotopos/GoToPos.h"
#include "skills/Attack.h"
#include "include/roboteam_ai/treeinterp/MidFieldHarassRole.h"

namespace bt {


    std::shared_ptr<Role> MidFieldHarassRole::createMidFieldHarassRole(std::string name) {
        /// Create the role tree here
        std::shared_ptr<bt::Role> roleNode = std::make_shared<bt::Role>(name);
        bt::BeingPassedToHelper helper = bt::BeingPassedToHelper();
        auto beingPassedToNode = helper.createBeingPassedToChecker();

        /// TOP LAYER:
        std::shared_ptr<bt::Selector> select = std::make_shared<bt::Selector>();
        auto localbb = std::make_shared<bt::Blackboard>();
        localbb->setString("ROLE", name);
        auto temp2 = std::make_shared<RobotOutOfFieldHelper>();
        auto outOfFieldNode = temp2->createRobotOutOfFieldHelper();


        /// Children of select node
        select->addChild(beingPassedToNode);
        select->addChild(outOfFieldNode);


        /// Give all of the constructed nodes to the role node, set its role, and return this role node.
        roleNode->addChild(select);
        roleNode->setRole(name);
        return roleNode;
    }

    MidFieldHarassRole::MidFieldHarassRole() {

    }


}
