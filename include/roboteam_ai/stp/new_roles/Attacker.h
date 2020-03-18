//
// Created by jordi on 17-03-20.
//

#ifndef RTT_ATTACKER_H
#define RTT_ATTACKER_H

#include "include/roboteam_ai/stp/Role.hpp"

namespace rtt::ai::stp::role {

class Attacker : public Role {
public:
    /**
     * Ctor that sets the name of the role and creates a statemachine of tactics
     * @param name name of the role
     */
    Attacker(std::string name);

private:
    StpInfo calculateInfoForTactic(StpInfo const &info) noexcept override;

    bool shouldRoleReset(const StpInfo &info) noexcept override;

    /**
     * Calculate position of the goal to kick to
     * @param info Info of the Role
     * @return Position to kick to
     */
    Vector2 calculateKickPosition(StpInfo const info) noexcept;
};

} // namespace rtt::ai::stp::role

#endif //RTT_ATTACKER_H
