//
// Created by jessevw on 03.12.19.
//

#ifndef RTT_PASSROLE_H
#define RTT_PASSROLE_H

#include <treeinterp/tactics/DefaultTactic.h>
#include "bt/BehaviorTree.h"
#include "bt/Role.h"

namespace bt {

class PassRole {
   public:
    /**
     * Creates a pass role behaviour tree. This tree is created in BTFactory.
     * @return the behaviour tree that contains a pass role
     */
    std::shared_ptr<Role> createPassRole(std::string rolename);

   private:
    /**
     * Vector that is used by RobotDealer to determine how to match robots to robot IDs.
     */
    std::vector<std::pair<std::string, rtt::ai::robotDealer::RobotType>> robots;
};
}  // namespace bt
#endif  // RTT_PASSROLE_H
