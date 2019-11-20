//
// Created by jessevw on 19.11.19.
//
#include <include/roboteam_ai/bt/composites/Selector.hpp>
#include <include/roboteam_ai/conditions/IsInDefenseArea.hpp>
#include <include/roboteam_ai/skills/Receive.h>
#include "bt/RoleDivider.h"
#include "bt/BehaviorTree.hpp"
#include "bt/decorators/Repeater.hpp"
#include "treeinterp/TreeProtoType.h"
#include "bt/tactics/DefaultTactic.h"
#include "bt/Role.h"
#include "skills/gotopos/GoToPos.h"
#include "skills/Attack.h"
#include "include/roboteam_ai/TreeHelper/BeingPassedToHelper.h"

namespace bt {
    BeingPassedToHelper::BeingPassedToHelper() {}

    std::shared_ptr<bt::Node> BeingPassedToHelper::createBeingPassedToChecker() {
        std::shared_ptr<bt::Selector> select;
        std::shared_ptr<rtt::ai::Receive> receive;
        std::shared_ptr<rtt::ai::IsInDefenseArea> defenseArea;
        select->addChild(defenseArea);
        select->addChild(receive);
        return select;
    }
}