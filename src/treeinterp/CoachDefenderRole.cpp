//
// Created by jordi on 03-12-19.
//

#include "include/roboteam_ai/treeinterp/CoachDefenderRole.h"
#include <include/roboteam_ai/TreeHelper/BeingPassedToHelper.h>
#include <include/roboteam_ai/TreeHelper/RobotOutOfFieldHelper.h>
#include <include/roboteam_ai/conditions/IsOnPassLine.h>
#include <include/roboteam_ai/skills/AvoidBall.h>
#include <include/roboteam_ai/bt/composites/Sequence.hpp>
#include <include/roboteam_ai/conditions/ShouldHandleBall.h>
#include <include/roboteam_ai/bt/decorators/Inverter.hpp>
#include <include/roboteam_ai/conditions/IsInDefenseArea.hpp>
#include <include/roboteam_ai/conditions/BallOutOfField.h>
#include <include/roboteam_ai/bt/composites/MemSelector.hpp>
#include <include/roboteam_ai/skills/Pass.h>
#include <include/roboteam_ai/skills/ChipForward.h>
#include <include/roboteam_ai/skills/CoachDefend.h>
#include <include/roboteam_ai/bt/composites/Selector.hpp>

namespace bt {

    std::shared_ptr<Role> bt::CoachDefenderRole::createCoachDefenderRole(std::string rolename) {
        /// Create the role tree
        auto roleNode = std::make_shared<bt::Role>(rolename);
        roleNode->setRole(rolename);
        auto roleBb = std::make_shared<bt::Blackboard>();
        roleBb->setString("ROLE", rolename);

        /// Is being passed to logic
        auto beingPassedToHelper = bt::BeingPassedToHelper();
        auto beingPassedToSequence = beingPassedToHelper.createBeingPassedToChecker();

        /// Robot outside field logic
        auto outOfFieldHelper = std::make_shared<RobotOutOfFieldHelper>();
        auto outOfFieldSequence = outOfFieldHelper->createRobotOutOfFieldHelper();

        /// Is on pass line logic
        auto isOnPassLineNode = std::make_shared<rtt::ai::IsOnPassLine>("isOnPassLine", roleBb);
        auto avoidBallBb = std::make_shared<bt::Blackboard>();
        avoidBallBb->setString("type", "passing");
        auto avoidBallNode = std::make_shared<rtt::ai::AvoidBall>("avoidBall", avoidBallBb);
        std::vector<std::shared_ptr<bt::Node>> nodes = {isOnPassLineNode, avoidBallNode};
        auto isOnPassLineSequence = std::make_shared<bt::Sequence>(nodes);

        /// Should handle ball logic
        auto shouldHandleBallNode = std::make_shared<rtt::ai::ShouldHandleBall>("shouldHandleBall", roleBb);

        auto inverterBallOutOfField = std::make_shared<bt::Inverter>();
        auto ballOutOfFieldNode = std::make_shared<rtt::ai::BallOutOfField>("ballOutOfField", roleBb);
        inverterBallOutOfField->addChild(ballOutOfFieldNode);

        auto inverterIsInDefenseArea = std::make_shared<bt::Inverter>();
        auto isInDefenseAreaNode = std::make_shared<rtt::ai::IsInDefenseArea>("isInDefenseArea", roleBb);
        inverterIsInDefenseArea->addChild(isInDefenseAreaNode);

        auto passSkill = std::make_shared<rtt::ai::Pass>("Pass", roleBb);
        auto chipForwardSkill = std::make_shared<rtt::ai::ChipForward>("Chip forward", roleBb);
        auto memSelector = std::make_shared<bt::MemSelector>();
        memSelector->addChild(passSkill);
        memSelector->addChild(chipForwardSkill);

        std::vector<std::shared_ptr<bt::Node>> shouldHandleBallNodes =
                {shouldHandleBallNode, inverterBallOutOfField, inverterIsInDefenseArea, memSelector};
        auto shouldHandleBallSequence = std::make_shared<bt::Sequence>(shouldHandleBallNodes);

        /// Coach defend skill
        auto coachDefendSkill = std::make_shared<rtt::ai::CoachDefend>("Coach defend", roleBb);

        /// Connect all nodes
        std::vector<std::shared_ptr<bt::Node>> coachDefenderNodes =
                {beingPassedToSequence, outOfFieldSequence, isOnPassLineSequence, shouldHandleBallSequence, coachDefendSkill};
        auto selector = std::make_shared<bt::Selector>(coachDefenderNodes);
        roleNode->addChild(selector);

        return roleNode;
    }

    bt::CoachDefenderRole::CoachDefenderRole() {

    }

}