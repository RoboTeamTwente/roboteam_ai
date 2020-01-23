//
// Created by jesse on 19.10.19.
//

#include "include/roboteam_ai/treeinterp/MidFieldHarassRole.h"

#include <include/roboteam_ai/treehelpers/BeingPassedToHelper.h>
#include <include/roboteam_ai/treehelpers/RobotOutOfFieldHelper.h>
#include <include/roboteam_ai/conditions/BallOutOfField.h>
#include <include/roboteam_ai/conditions/HasClearShot.h>
#include <include/roboteam_ai/conditions/IsOnPassLine.h>
#include <include/roboteam_ai/conditions/ShouldHandleBall.h>
#include <include/roboteam_ai/skills/AvoidBall.h>
#include <include/roboteam_ai/skills/MidFieldHarasser.h>
#include <include/roboteam_ai/skills/Pass.h>
#include <include/roboteam_ai/treeinterp/PassRole.h>

#include <include/roboteam_ai/bt/composites/MemSelector.hpp>
#include <include/roboteam_ai/bt/composites/Selector.hpp>
#include <include/roboteam_ai/bt/composites/Sequence.hpp>
#include <include/roboteam_ai/bt/decorators/Inverter.hpp>
#include <include/roboteam_ai/conditions/IsInDefenseArea.hpp>

#include "bt/Role.h"
#include "skills/Attack.h"
#include "skills/gotopos/GoToPos.h"

namespace bt {

std::shared_ptr<Role> MidFieldHarassRole::createMidFieldHarassRole(std::string name) {
    /// Create the role tree here
    std::shared_ptr<bt::Role> roleNode = std::make_shared<bt::Role>(name);
    bt::BeingPassedToHelper helper = bt::BeingPassedToHelper();
    auto beingPassedToNode = helper.createBeingPassedToChecker();

    /// TOP LAYER:
    std::shared_ptr<bt::Selector> select;
    auto localbb = std::make_shared<bt::Blackboard>();
    localbb->setString("ROLE", name);
    auto temp2 = std::make_shared<RobotOutOfFieldHelper>();
    auto outOfFieldNode = temp2->createRobotOutOfFieldHelper();

    /// Is on pass line logic
    std::shared_ptr<Sequence> passSequence;
    auto isOnPassLine = std::make_shared<rtt::ai::IsOnPassLine>("IsOnPassLine", localbb);

    auto avoidBallbb = std::make_shared<bt::Blackboard>();
    avoidBallbb->setString("type", "passing");
    auto avoidBall = std::make_shared<rtt::ai::AvoidBall>(name, avoidBallbb);

    passSequence = std::make_shared<bt::Sequence>(nvector{isOnPassLine, avoidBall});

    /// Should handle ball logic
    std::shared_ptr<Sequence> shouldHandleBallSeq;
    auto shouldHandleBall = std::make_shared<rtt::ai::ShouldHandleBall>("shoudhandleball", localbb);

    auto inverterIsInDefenseArea = std::make_shared<bt::Inverter>();
    auto isIndefenseArea = std::make_shared<rtt::ai::IsInDefenseArea>("isInDefenseArea", localbb);

    auto inverterBallOutOfField = std::make_shared<bt::Inverter>();
    auto ballOutOfField = std::make_shared<rtt::ai::BallOutOfField>("ballOutOfField", localbb);

    auto memSelector = std::make_shared<bt::MemSelector>();

    std::shared_ptr<Sequence> clearShotSequence;
    auto hasClearShot = std::make_shared<rtt::ai::HasClearShot>("hasclearshot", localbb);
    auto attack = std::make_shared<rtt::ai::Attack>(name, localbb);

    auto passSkill = std::make_shared<rtt::ai::Pass>("Pass", localbb);
    auto attackPassSkill = std::make_shared<rtt::ai::Attack>("Pass attack", localbb);
    auto passAttackSequence = std::make_shared<bt::Sequence>();

    passAttackSequence->addChild(passSkill);
    passAttackSequence->addChild(attackPassSkill);

    shouldHandleBallSeq = std::make_shared<bt::Sequence>(nvector{shouldHandleBall, inverterIsInDefenseArea, inverterBallOutOfField, memSelector});

    inverterBallOutOfField->addChild(ballOutOfField);
    inverterIsInDefenseArea->addChild(isIndefenseArea);

    clearShotSequence = std::make_shared<Sequence>(nvector{hasClearShot, attack});

    memSelector->addChild(clearShotSequence);
    memSelector->addChild(passAttackSequence);

    auto midFieldHarassSkill = std::make_shared<rtt::ai::MidFieldHarasser>("Midfield harasser", localbb);

    /// Children of select node
    select = std::make_shared<Selector>(nvector{beingPassedToNode, outOfFieldNode, passSequence, shouldHandleBallSeq, midFieldHarassSkill});

    /// Give all of the constructed nodes to the role node, set its role, and return this role node.
    roleNode->addChild(select);
    roleNode->setRoleString(name);
    return roleNode;
}

}  // namespace bt
