//
// Created by jessevw on 19.11.19.
//
#include <include/roboteam_ai/bt/composites/Selector.hpp>
#include <include/roboteam_ai/conditions/IsInDefenseArea.hpp>
#include <include/roboteam_ai/skills/Receive.h>
#include <include/roboteam_ai/bt/composites/Sequence.hpp>
#include <include/roboteam_ai/conditions/IsBeingPassedTo.h>
#include "bt/RoleDivider.h"
#include "bt/tactics/DefaultTactic.h"
#include "bt/Role.h"
#include "skills/gotopos/GoToPos.h"
#include "skills/Attack.h"
#include "include/roboteam_ai/TreeHelper/BeingPassedToHelper.h"

namespace bt {
    BeingPassedToHelper::BeingPassedToHelper() {}

    std::shared_ptr<bt::Node> BeingPassedToHelper::createBeingPassedToChecker() {
        auto localbb = std::make_shared<bt::Blackboard>();

        std::shared_ptr<bt::Sequence> seque = std::make_shared<bt::Sequence>();
        std::shared_ptr<rtt::ai::Receive> receive = std::make_shared<rtt::ai::Receive>("receivernode", localbb);
        std::shared_ptr<rtt::ai::IsBeingPassedTo> beingPassedTo = std::make_shared<rtt::ai::IsBeingPassedTo>("being passed to", localbb);
        seque->addChild(beingPassedTo);
        seque->addChild(receive);
        return seque;
    }
}