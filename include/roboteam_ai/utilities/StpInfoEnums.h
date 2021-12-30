//
// Created by alexander on 03-12-21.
//

#ifndef RTT_STPINFOENUMS_H
#define RTT_STPINFOENUMS_H

namespace rtt::ai::stp {
/**
 * The distance the robot should block at
 */
enum class BlockDistance { CLOSE = 1, HALFWAY, FAR };

/**
 * Whether this robot should kick or chip in the shoot skill
 */
enum class KickOrChip { KICK, CHIP };

/**
 * The PIDType to be used by this robot for path planning/tracking
 */
enum class PIDType { DEFAULT, RECEIVE, INTERCEPT, KEEPER, KEEPER_INTERCEPT };

/**
 * The type of shot this robot should use. Used for determining kick/chip velocity
 */
enum class ShotType { PASS, TARGET, MAX };

/**
 * The status that a skill/tactic can return
 */
enum class Status { Waiting, Success, Failure, Running };

/**
 * The AvoidObjects struct containing what the robot should avoid
 */
struct AvoidObjects {
    bool shouldAvoidBall = false;
    bool shouldAvoidDefenseArea = true;
    bool shouldAvoidOutOfField = true;
};
}  // namespace rtt::ai::stp
#endif  // RTT_STPINFOENUMS_H
