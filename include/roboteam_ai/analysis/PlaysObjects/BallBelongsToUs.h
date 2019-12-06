//
// Created by jessevw on 06.12.19.
//

#ifndef RTT_BALLBELONGSTOUS_H
#define RTT_BALLBELONGSTOUS_H

namespace rtt::ai::analysis {
    /**
     * Invariant that is true when the ball belongs to us
     */
    class BallBelongsToUs : public Invariant {
    public:
        /**
         * Functional implementation of when the ball belongs to us
         * @param world the current world state
         * @param field
         * @return true if the ball belongs to us, false otherwise
         */
        bool isTrue(rtt::ai::world::World* world, rtt::ai::world::Field* field);
    };
}

#endif //RTT_BALLBELONGSTOUS_H
