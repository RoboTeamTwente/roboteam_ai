//
// Created by jessevw on 03.12.19.
//

#include "include/roboteam_ai/treeinterp/SideAttackerRole.h"

#include <include/roboteam_ai/skills/Pass.h>
#include <include/roboteam_ai/skills/Halt.h>
#include <include/roboteam_ai/bt/composites/MemSequence.hpp>
#include <include/roboteam_ai/bt/composites/Selector.hpp>
#include <include/roboteam_ai/bt/composites/Sequence.hpp>
#include "include/roboteam_ai/treeinterp/PassRole.h"

#include "bt/BehaviorTree.hpp"
#include "bt/decorators/Repeater.hpp"
#include "treeinterp/OffensiveStrategy.h"
#include "bt/tactics/DefaultTactic.h"
#include "bt/Role.h"
#include "skills/gotopos/GoToPos.h"
#include "skills/Attack.h"

namespace bt {

    /**
     * Creates pass role. This role passes if there is a pass available, otherwise it halts
     * @param rolename the name of the role (must correspond to the rolename in the robots vector of the tactic
     * @return role for passing and then halting
     */
    std::shared_ptr<Role> SideAttackerRole::createSideAttackerRole(std::string rolename) {
        auto select = std::make_shared<bt::Selector>();
        auto sequence = std::make_shared<bt::Sequence>();
        auto reflectKickSelector = std::make_shared<bt::Selector>();
        // selector

        // sequence
        // is being passed to
        // selector
        //      memseq
        //      canreflectkick
        //      reflectkick
        //
        //      receive
        return std::shared_ptr<Role>();

    }
}