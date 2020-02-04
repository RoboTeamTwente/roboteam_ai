//
// Created by jessevw on 19.11.19.
//
#include <include/roboteam_ai/conditions/RobotOutside.h>
#include <include/roboteam_ai/skills/gotopos/GTPSpecial.h>
#include <include/roboteam_ai/treehelpers/RobotOutOfFieldHelper.h>

#include <src/bt/include/bt/composites/Sequence.h>

#include "bt/Role.h"
#include "bt/RoleDivider.h"
#include "skills/Attack.h"

// Reference wrapper

namespace bt {
RobotOutOfFieldHelper::RobotOutOfFieldHelper() {}
std::shared_ptr<bt::Node> RobotOutOfFieldHelper::createRobotOutOfFieldHelper() {
    auto localbb = std::make_shared<bt::Blackboard>();
    auto robotOutsideBB = std::make_shared<bt::Blackboard>();

    robotOutsideBB->setString("type", "defaultType");

    std::shared_ptr<bt::Sequence> sequence = std::make_shared<bt::Sequence>();
    std::shared_ptr<rtt::ai::RobotOutside> robotOutside = std::make_shared<rtt::ai::RobotOutside>("RobotOutside", localbb);
    std::shared_ptr<rtt::ai::GTPSpecial> gtp = std::make_shared<rtt::ai::GTPSpecial>("GoToPosMidfield", robotOutsideBB);

    sequence->addChild(robotOutside);
    sequence->addChild(gtp);
    return sequence;
}
}  // namespace bt