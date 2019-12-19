//
// Created by jessevw on 03.12.19.
//

#include "include/roboteam_ai/treeinterp/SideAttackerRole.h"

#include <include/roboteam_ai/skills/Pass.h>
#include <include/roboteam_ai/bt/composites/MemSequence.hpp>
#include <include/roboteam_ai/bt/composites/Selector.hpp>
#include <include/roboteam_ai/bt/composites/Sequence.hpp>
#include <include/roboteam_ai/conditions/IsBeingPassedTo.h>
#include <include/roboteam_ai/conditions/CanReflectKick.h>
#include <include/roboteam_ai/skills/ReflectKick.h>
#include <include/roboteam_ai/skills/Receive.h>
#include "bt/Role.h"
#include "skills/Attack.h"
#include <include/roboteam_ai/TreeHelper/RobotOutOfFieldHelper.h>
#include <include/roboteam_ai/conditions/IsOnPassLine.h>
#include <include/roboteam_ai/skills/AvoidBall.h>
#include <include/roboteam_ai/conditions/ShouldHandleBall.h>
#include <include/roboteam_ai/conditions/BallOutOfField.h>
#include <include/roboteam_ai/conditions/IsInDefenseArea.hpp>
#include <include/roboteam_ai/bt/decorators/Inverter.hpp>
#include <include/roboteam_ai/conditions/HasClearShot.h>
#include <include/roboteam_ai/skills/GetBall.h>
#include <include/roboteam_ai/skills/DribbleForward.h>
#include <include/roboteam_ai/skills/SideAttacker.h>

namespace bt {

    /**
     * Creates side attacker role. This role is used as offense role. When building roles it is important to remember to build the nodes first, then connect them to the tree.
     * Otherwise prepare for segfaults. When segfaults are experienced, keep in mind to check (in the debugger) whether all the nodes are added properly. Often one (or multiple) nodes are
     * nullptrs because they are added improperly and this causes segfaults in the interface
     * @param rolename the name of the role (must correspond to the rolename in the robots vector of the tactic
     * @return role for passing and then halting
     */
    std::shared_ptr<Role> SideAttackerRole::createSideAttackerRole(std::string rolename) {
        std::shared_ptr<bt::Selector> select;
        std::shared_ptr<bt::Sequence> seq;
        std::shared_ptr<bt::Selector> reflectKickSelect;
        std::shared_ptr<bt::MemSequence> reflectKickMemSeq;

        auto localbb = std::make_shared<bt::Blackboard>();

        /// Leftmost tree nodes
        auto beingPassedTo = std::make_shared<rtt::ai::IsBeingPassedTo>("is being passed to", localbb);
        auto canReflectKick = std::make_shared<rtt::ai::CanReflectKick>("can reflect kick", localbb);
        auto reflectKick = std::make_shared<rtt::ai::ReflectKick>("reflectkick", localbb);
        auto receive = std::make_shared<rtt::ai::Receive>("receive", localbb);

        nvector order3 { canReflectKick, reflectKick };
        reflectKickMemSeq = std::make_shared<bt::MemSequence>(order3);

        nvector order2 { reflectKickMemSeq, receive };
        reflectKickSelect = std::make_shared<bt::Selector>(order2);

        nvector order{ beingPassedTo, reflectKickSelect };
        seq = std::make_shared<bt::Sequence>(order);

        /// Making middle sequences
        auto outOfFieldHelper = bt::RobotOutOfFieldHelper();
        auto outOfFieldLogic = outOfFieldHelper.createRobotOutOfFieldHelper();

        std::shared_ptr<Sequence> passLineSequence;
        auto isOnPassLine = std::make_shared<rtt::ai::IsOnPassLine>("is on pass line", localbb);
        auto avoidBall = std::make_shared<rtt::ai::AvoidBall>("avoiding the ball", localbb);


        passLineSequence = std::make_shared<bt::Sequence>(nvector{isOnPassLine, avoidBall});

        /// Make big right tree here
        /// Start by making all the nodes
        auto defenseAreaBB = std::make_shared<bt::Blackboard>();
        defenseAreaBB->setBool("useRobot", false);
        defenseAreaBB->setBool("outsideField", true);
        defenseAreaBB->setBool("outsideField", true);
        defenseAreaBB->setDouble("secondsAhead", 0.4);

        auto ballOutOfFieldBB = std::make_shared<bt::Blackboard>();
        ballOutOfFieldBB->setDouble("secondsAhead", 0.4);

        auto shouldHandleBall = std::make_shared<rtt::ai::ShouldHandleBall>("should handle ball", localbb);
        auto isInDefenseArea = std::make_shared<rtt::ai::IsInDefenseArea>("is in defense area", defenseAreaBB);
        auto ballOutOfField = std::make_shared<rtt::ai::BallOutOfField>("ball out of field", ballOutOfFieldBB);
        auto ballOutOfFieldInverter = std::make_shared<bt::Inverter>();
        auto IsInDefenseAreaInverter = std::make_shared<bt::Inverter>();
        IsInDefenseAreaInverter->addChild(isInDefenseArea);
        ballOutOfFieldInverter->addChild(ballOutOfField);
        auto hasClearShot = std::make_shared<rtt::ai::HasClearShot>("has clear shot", localbb);
        auto attack = std::make_shared<rtt::ai::Attack>("attack", localbb);
        auto pass = std::make_shared<rtt::ai::Pass>("pass", localbb);
        auto getBall = std::make_shared<rtt::ai::GetBall>("get ball", localbb);

        auto dribbleBB = std::make_shared<bt::Blackboard>();
        dribbleBB->setDouble("dribbleDistance", 0.59);

        auto dribbleForward = std::make_shared<rtt::ai::DribbleForward>("dribble forward", dribbleBB);
        auto dribblePass = std::make_shared<rtt::ai::Pass>("pass after dribble", localbb);

        auto dribbleMemSequence = std::make_shared<bt::MemSequence>(nvector { getBall, dribbleForward, dribblePass });
        auto dribbleMemSelector = std::make_shared<bt::MemSequence>(nvector { dribblePass, dribbleMemSequence });
        auto clearShotSequence = std::make_shared<bt::Sequence>(nvector { hasClearShot, attack });
        auto clearShotAndAttackSelector = std::make_shared<bt::Selector>(nvector {clearShotSequence, dribbleMemSelector });
        auto rightSequence = std::make_shared<bt::Sequence>(nvector {shouldHandleBall, IsInDefenseAreaInverter, ballOutOfFieldInverter, clearShotAndAttackSelector });

        auto sideAttacker = std::make_shared<rtt::ai::SideAttacker>("side attacker", localbb);
        
        auto roleNode = std::make_shared<Role>(rolename);
        select = std::make_shared<bt::Selector>(nvector { seq, outOfFieldLogic, passLineSequence, rightSequence, sideAttacker });
        roleNode->addChild(select);
        roleNode->setRoleString(rolename);
        std::cout << (roleNode->getChildren().size() > 0) << std::endl;
        return roleNode;

    }
}