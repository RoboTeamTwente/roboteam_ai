//
// Created by doormat on 17-11-22.
//

#ifndef RTT_CHIPPINGPASS_H
#define RTT_CHIPPINGPASS_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class ChippingPass : public Role {
   public:
    /**
     * Ctar that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
     ChippingPass(std::string name);
};
} // namespace rtt::ai::stp::role

#endif  // RTT_CHIPPINGPASS_H
