//
// Created by timovdk on 3/10/20.
//

#ifndef RTT_TESTROLE_H
#define RTT_TESTROLE_H

#include "stp/Role.hpp"

namespace rtt::ai::stp {

class TestRole : public Role {
   public:
    /**
     * Ctor that sets the name of the role and creates a statemachine of tactics
     * @param name name of the role
     */
    TestRole(std::string name);
};
}  // namespace rtt::ai::stp

#endif  // RTT_TESTROLE_H
