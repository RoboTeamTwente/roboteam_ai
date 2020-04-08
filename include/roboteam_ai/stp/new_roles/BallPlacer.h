//
// Created by RobotJesse
//

#ifndef RTT_ATTACKER_H
#define RTT_ATTACKER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

    class BallPlacer : public Role {
    public:
        /**
         * Ctor that sets the name of the role and creates a statemachine of tactics
         * @param name name of the role
         */
        BallPlacer(std::string name);
    };

} // namespace rtt::ai::stp::role

#endif //RTT_ATTACKER_H
