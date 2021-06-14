//
// Created by jordi on 17-03-20.
//

#ifndef RTT_ATTACKER_H
#define RTT_ATTACKER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class Attacker : public Role {
   public:
    /**
     * Ctor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    Attacker(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_ATTACKER_H
