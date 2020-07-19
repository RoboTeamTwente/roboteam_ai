//
// Created by jessevw on 17.03.20.
//

#ifndef RTT_PASSER_H
#define RTT_PASSER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class Passer : public Role {
   public:
    /**
     * Ctor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    Passer(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_PASSER_H
