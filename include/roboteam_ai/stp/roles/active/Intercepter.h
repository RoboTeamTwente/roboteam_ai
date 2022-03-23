
//
// Created by sarah on 02-03-22.
//

#ifndef RTT_INTERCEPTER_H
#define RTT_INTERCEPTER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class Intercepter : public Role {
   public:
    /**
     * Ctor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    Intercepter(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_INTERCEPTER_H

