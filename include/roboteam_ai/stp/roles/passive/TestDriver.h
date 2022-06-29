//
// Created by agata on 29/06/2022.
//

#ifndef RTT_TESTDRIVER_H
#define RTT_TESTDRIVER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class TestDriver : public Role {
   public:
    /**
     * Ctor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    TestDriver(std::string name);

    [[nodiscard]] Status update(StpInfo const& info) noexcept override;
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_TESTDRIVER_H
