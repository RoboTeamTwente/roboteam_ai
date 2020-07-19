//
// Created by jordi on 24-03-20.
//

#ifndef RTT_ATTACK_H
#define RTT_ATTACK_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

class Attack : public Play {
   public:
    /**
     * Constructor that initializes roles with roles that are necessary for this play
     */
    Attack();

    /**
     * Gets the score for the current play
     *
     * On the contrary to isValidPlay() this checks how good the play actually is
     * return in range of 0 - 100
     *
     * @param world World to get the score for (world_new::World::instance())
     * @return The score, 0 - 100
     */
    uint8_t score(world_new::World *world) noexcept override;

    /**
     * Assigns robots to roles of this play
     */
    Dealer::FlagMap decideRoleFlags() const noexcept override;

    /**
     * Calculates info for the roles
     */
    void calculateInfoForRoles() noexcept override;

    /**
     * Gets the play name
     */
    const char *getName() override;

   protected:
    /**
     * This function is used to determine if -- when a role is in an endTactic -- the endTactic should be skipped.
     * An example could be BlockRobot and Intercept. You block a robot (endTactic) until a ball is shot and then the robot
     * closest to the ball should try to intercept (skip the BlockRobot tactic to execute Intercept)
     */
    bool shouldRoleSkipEndTactic() override;

   private:
    /**
     * Calculate point in goal to aim for
     * @return Target point
     */
    Vector2 calculateGoalTarget() noexcept;

    /**
     * Calculate points we want to aim for
     * @param field Field
     * @param fromPoint Position to shoot from
     * @return Line between the two aim points
     */
    Line getAimPoints(const world::Field &field, const Vector2 &fromPoint);

    /**
     * Returns the longest line from openSegments
     * @param openSegments Vector of lines
     * @return Longest line from openSegments
     */
    const Line &getLongestSegment(const std::vector<Line> &openSegments);
};

}  // namespace rtt::ai::stp::play

#endif  // RTT_ATTACK_H
