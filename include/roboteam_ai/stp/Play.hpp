//
// Created by john on 3/9/20.
//

#ifndef RTT_PLAY_HPP
#define RTT_PLAY_HPP

#include <array>
#include "Role.hpp"

namespace rtt::ai::stp {

    class Play {
    public:
        constexpr static size_t ROBOT_COUNT = 11;

        [[nodiscard]] virtual Status update(TacticInfo const& info);

    protected:
        std::array<Role, ROBOT_COUNT> roles;
    };

} // namespace rtt::ai::stp

#endif //RTT_PLAY_HPP
