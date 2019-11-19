//
// Created by jesse on 19.10.19.
//

#include <include/roboteam_ai/TreeHelper/BeingPassedToHelper.h>
#include "bt/RoleDivider.h"
#include "bt/BehaviorTree.hpp"
#include "bt/decorators/Repeater.hpp"
#include "treeinterp/TreeProtoType.h"
#include "bt/tactics/DefaultTactic.h"
#include "bt/Role.h"
#include "skills/gotopos/GoToPos.h"
#include "skills/Attack.h"
#include "include/roboteam_ai/treeinterp/MidFieldHarassRoleCPP.h"

namespace bt {


    std::shared_ptr<Role> MidFieldHarassRole::createMidFieldHarassRole(std::string name) {
        /// Create the role tree here
        bt::BeingPassedToHelper helper;
        auto passSubTree = helper.createBeingPassedToChecker();

    }



}
