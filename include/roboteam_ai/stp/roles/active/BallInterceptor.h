//
// Created by alexander on 12-05-22.
//

#ifndef RTT_BALLINTERCEPTOR_H
#define RTT_BALLINTERCEPTOR_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class BallInterceptor : public Role {
   public:
    /**
    * Ctor that sets the name of the role and creates a state machine of tactics
    * @param name name of the role
    */
    BallInterceptor(std::string name);
};
}  // namespace rtt::ai::stp::role


#endif  // RTT_BALLINTERCEPTOR_H
