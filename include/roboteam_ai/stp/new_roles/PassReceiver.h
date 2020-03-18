//
// Created by jessevw on 17.03.20.
//

#ifndef RTT_PASSRECEIVER_H
#define RTT_PASSRECEIVER_H

#include <stp/Role.hpp>

namespace rtt::ai::stp {

    class PassReceiver : public Role {
    public:
        /**
         * Ctor that sets the name of the role and creates a statemachine of tactics
         * @param name name of the role
         */
        PassReceiver(std::string name);

    protected:
        StpInfo calculateInfoForTactic(StpInfo const &info) noexcept override;

        bool shouldRoleReset(const StpInfo &info) noexcept override;
    };
}  // namespace rtt::ai::stp


#endif //RTT_PASSRECEIVER_H
