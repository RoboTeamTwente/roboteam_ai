//
// Created by tijmen on 14-07-22.
//

#ifndef RTT_KEEPERPASSER_H
#define RTT_KEEPERPASSER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class KeeperPasser : public Role {
   public:
    /**
     * Ctor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    KeeperPasser(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_KEEPERPASSER_H
