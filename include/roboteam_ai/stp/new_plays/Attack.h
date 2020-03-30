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
    Attack(std::string playName);

    bool isValidPlayToStart(world_new::World* world) noexcept override;

    /**
     * Gets the score for the current play
     *
     * On the contrary to isValidPlay() this checks how good the play actually is
     * return in range of 0 - 100
     *
     * @param world World to get the score for (world_new::World::instance())
     * @return The score, 0 - 100
     */
    uint8_t score(world_new::World* world) noexcept override;

    /**
     * Assigns robots to roles of this play
     */
    Dealer::FlagMap decideRoleFlags() const noexcept override;

    /**
     * Calculates info for the roles
     */
    void calculateInfoForRoles() noexcept override;

protected:
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
    Line getAimPoints(const Field &field, const Vector2 &fromPoint);

    /**
     * Returns the longest line from openSegments
     * @param openSegments Vector of lines
     * @return Longest line from openSegments
     */
    const Line &getLongestSegment(const std::vector<Line> &openSegments);
};

} // namespace rtt::ai::stp::play

#endif //RTT_ATTACK_H
