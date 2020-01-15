//
// Created by jessevw on 19.11.19.
//

#ifndef RTT_MIDFIELDHARASSROLE_H
#define RTT_MIDFIELDHARASSROLE_H
#include <include/roboteam_ai/bt/tactics/DefaultTactic.h>
#include "bt/BehaviorTree.hpp"
#include "bt/Role.h"


namespace bt {

    class MidFieldHarassRole {
    public:
        /**
         * Creates a Play behaviour tree. This tree is created in BTFactory.
         * @return the behaviour tree that contains this strategy
         */
        std::shared_ptr<Role> createMidFieldHarassRole(std::string rolename);

    private:
        /**
         * Vector that is used by RobotDealer to determine how to match robots to robot IDs.
         */
        std::vector<std::pair<std::string, rtt::ai::robotDealer::RobotType>> robots;

    };
}


#endif //RTT_MIDFIELDHARASSROLE_H
