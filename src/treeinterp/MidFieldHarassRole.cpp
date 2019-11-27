//
// Created by jesse on 19.10.19.
//

#include <include/roboteam_ai/TreeHelper/BeingPassedToHelper.h>
#include <include/roboteam_ai/bt/composites/Selector.hpp>
#include <include/roboteam_ai/TreeHelper/RobotOutOfFieldHelper.h>
#include <include/roboteam_ai/bt/composites/Sequence.hpp>
#include <include/roboteam_ai/conditions/IsOnPassLine.h>
#include <include/roboteam_ai/skills/AvoidBall.h>
#include <include/roboteam_ai/bt/decorators/Inverter.hpp>
#include <include/roboteam_ai/conditions/IsInDefenseArea.hpp>
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


       /// Is on pass line logic
        auto passSeque = std::make_unique<bt::Sequence>();
        auto isOnPassLine = std::make_unique<rtt::ai::IsOnPassLine>(name, localbb);

        auto avoidBallbb = std::make_shared<bt::Blackboard>();
        avoidBallbb->setString("type", "passing");
        auto avoidBall = std::make_unique<rtt::ai::AvoidBall>(name, avoidBallbb);

        auto inverterIsInDefenseArea = std::make_shared<bt::Inverter>();
        auto isIndefenseArea = std::make_shared<rtt::ai::IsInDefenseArea>();

        auto inverterBallOutOfField = std::make_shared<bt::Inverter>();

        /// Should handle ball logic
        auto shouldHandleBallSeq = std::make_unique<bt::Sequence>();


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
