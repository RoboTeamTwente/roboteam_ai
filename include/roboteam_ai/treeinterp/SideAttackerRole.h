//
// Created by jessevw on 03.12.19.
//

#ifndef RTT_SIDEATTACKERROLE_H
#define RTT_SIDEATTACKERROLE_H

#include <include/roboteam_ai/bt/tactics/DefaultTactic.h>
#include "bt/BehaviorTree.hpp"
#include "bt/Role.h"

namespace bt {
    class SideAttackerRole {
        public:
        /**
         * Creates a pass role behaviour tree. This tree is created in BTFactory.
         * @return the behaviour tree that contains a pass role
         */
        std::shared_ptr<Role> createSideAttackerRole(std::string rolename);

        private:
        /**
         * Vector that is used by RobotDealer to determine how to match robots to robot IDs.
         */
        std::vector<std::pair<std::string, rtt::ai::robotDealer::RobotType>> robots;
    };
}  // namespace bt
#endif  // RTT_SIDEATTACKERROLE_H
