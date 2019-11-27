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
#include <include/roboteam_ai/conditions/BallOutOfField.h>
#include <include/roboteam_ai/conditions/ShouldHandleBall.h>
#include <include/roboteam_ai/bt/composites/MemSelector.hpp>
#include <include/roboteam_ai/conditions/HasClearShot.h>
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
        auto passSeque = std::make_shared<bt::Sequence>();
        auto isOnPassLine = std::make_shared<rtt::ai::IsOnPassLine>(name, localbb);

        auto avoidBallbb = std::make_shared<bt::Blackboard>();
        avoidBallbb->setString("type", "passing");
        auto avoidBall = std::make_shared<rtt::ai::AvoidBall>(name, avoidBallbb);
        passSeque->addChild(isOnPassLine);
        passSeque->addChild(avoidBall);

        /// Should handle ball logic
        auto shouldHandleBallSeq = std::make_shared<bt::Sequence>();
        auto shouldHandleBall = std::make_shared<rtt::ai::ShouldHandleBall>(name, localbb);

        auto inverterIsInDefenseArea = std::make_shared<bt::Inverter>();
        auto isIndefenseArea = std::make_shared<rtt::ai::IsInDefenseArea>(localbb);

        auto inverterBallOutOfField = std::make_shared<bt::Inverter>();
        auto ballOutOfField = std::make_shared<rtt::ai::BallOutOfField>();

        auto memSelector = std::make_shared<bt::MemSelector>();

        auto clearShotSeque = std::make_shared<bt::Sequence>();
        auto hasClearShot = std::make_shared<rtt::ai::HasClearShot>();
        auto attack = std::make_shared<rtt::ai::Attack>(name, localbb);

        shouldHandleBallSeq->addChild(shouldHandleBall);
        shouldHandleBallSeq->addChild(inverterIsInDefenseArea);
        shouldHandleBallSeq->addChild(inverterBallOutOfField);
        shouldHandleBallSeq->addChild(memSelector);

        inverterBallOutOfField->addChild(ballOutOfField);
        inverterIsInDefenseArea->addChild(isIndefenseArea);

        memSelector->addChild(clearShotSeque);
        //memSelector->addChild()

        clearShotSeque->addChild(hasClearShot);
        clearShotSeque->addChild(attack);


        /// Children of select node
        select->addChild(beingPassedToNode);
        select->addChild(outOfFieldNode);
        select->addChild(passSeque);
        select->addChild(shouldHandleBallSeq);


        /// Give all of the constructed nodes to the role node, set its role, and return this role node.
        roleNode->addChild(select);
        roleNode->setRole(name);
        return roleNode;
    }

    MidFieldHarassRole::MidFieldHarassRole() {

    }


}
