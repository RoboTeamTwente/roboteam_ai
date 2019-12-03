//
// Created by jordi on 03-12-19.
//

#ifndef RTT_COACHDEFENDERROLE_H
#define RTT_COACHDEFENDERROLE_H
#include <include/roboteam_ai/bt/tactics/DefaultTactic.h>
#include "bt/Role.h"

namespace bt {

    class CoachDefenderRole {
    public:
        CoachDefenderRole();

        /**
         * Creates a Strategy behaviour tree. This tree is created in BTFactory.
         * @return the behaviour tree that contains this strategy
         */
        std::shared_ptr<Role> createCoachDefenderRole(std::string rolename);

    private:
        /**
         * Vector that is used by RobotDealer to determine how to match robots to robot IDs.
         */
        std::vector<std::pair<std::string, rtt::ai::robotDealer::RobotType>> robots;
    };

}


#endif //RTT_COACHDEFENDERROLE_H
