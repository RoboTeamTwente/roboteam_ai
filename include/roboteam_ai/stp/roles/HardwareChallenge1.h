//
// Created by floris on 08-06-21.
//

#ifndef RTT_HardwareChallenge1_H
#define RTT_HardwareChallenge1_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class HardwareChallenge1 : public Role {
   public:
    /**
     * Ctor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    HardwareChallenge1(std::string name);
};
}  // namespace rtt::ai::stp::role


#endif  // RTT_HardwareChallenge1_H
