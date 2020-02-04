//
// Created by john on 12/17/19.
//

#ifndef RTT_TEAM_HPP
#define RTT_TEAM_HPP

namespace rtt::world_new {
/**
 * Enum used for indicating team
 */
enum Team : short {
    // our team
    us,
    // their team
    them,
    /**
     * If a robot has both as team -> invalid.
     */
    both
};
}  // namespace rtt::world_new

#endif  // RTT_TEAM_HPP
