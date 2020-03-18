//
// Created by jessevw on 17.03.20.
//

#ifndef RTT_PASSER_H
#define RTT_PASSER_H

#include <stp/Role.hpp>

namespace rtt::ai::stp {

    class Passer : public Role {
    public:
        /**
         * Ctor that sets the name of the role and creates a statemachine of tactics
         * @param name name of the role
         */
        Passer(std::string name);

        bool shouldRoleReset(const StpInfo &info) noexcept override;

        StpInfo calculateInfoForTactic(StpInfo const &info) noexcept override;


    };
}  // namespace rtt::ai::stp

#endif //RTT_PASSER_H
