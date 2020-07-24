//
// Created by timovdk on 3/27/20.
//

#ifndef RTT_FORMATION_H
#define RTT_FORMATION_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class Formation : public Role {
   public:
    /**
     * Ctor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    Formation(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_FORMATION_H
