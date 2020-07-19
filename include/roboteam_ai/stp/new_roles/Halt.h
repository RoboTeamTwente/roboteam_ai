//
// Created by jessevw on 24.03.20.
//

#ifndef RTT_HALT_H
#define RTT_HALT_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class Halt : public Role {
   public:
    /**
     * Ctor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    Halt(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_HALT_H
