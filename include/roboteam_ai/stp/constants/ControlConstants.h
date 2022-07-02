//
// Created by jessevw on 25.03.20.
//

#ifndef RTT_CONTROLCONSTANTS_H
#define RTT_CONTROLCONSTANTS_H

#include <cstdint>

namespace rtt::ai::stp::control_constants {
// Kick and Chip constants
extern const double MAX_KICK_POWER;
extern const double MIN_KICK_POWER;
extern const double MAX_CHIP_POWER;
extern const double MIN_CHIP_POWER;
extern const double MAX_POWER_KICK_DISTANCE;
extern const double MAX_POWER_CHIP_DISTANCE;
extern const double MAX_KICK_ATTEMPTS;
extern const double MAX_CHIP_ATTEMPTS;

// Robot physical constants
extern const double ROBOT_RADIUS;
extern const double CENTER_TO_FRONT;

// Dribbler constants
extern const double TURN_ON_DRIBBLER_DISTANCE;

// Team constants
inline constexpr uint8_t MAX_ROBOT_COUNT = 11;

// Ball constants
extern const double BALL_STILL_VEL;
extern const double BALL_STILL_VEL2; // BALL_STILL_VELL but squared.
extern const double BALL_GOT_SHOT_LIMIT;
extern const double BALL_IS_MOVING_SLOW_LIMIT;
extern const double BALL_IS_CLOSE;
extern const double BALL_RADIUS;
extern const double HAS_KICKED_ERROR_MARGIN;
extern const double HAS_CHIPPED_ERROR_MARGIN;
extern const double ENEMY_CLOSE_TO_BALL_DISTANCE;

// RobotCommand limits
extern const double MAX_VEL_CMD;
extern const double MAX_DRIBBLER_CMD;
extern const double ANGLE_RATE;
extern const double MAX_VEL_WHEN_HAS_BALL;

// HasBall margins
extern const double HAS_BALL_ANGLE_ERROR_MARGIN;
extern const double HAS_BALL_DISTANCE_ERROR_MARGIN;

// GTP Constants
extern const double GO_TO_POS_ERROR_MARGIN;
extern const double GO_TO_POS_ANGLE_ERROR_MARGIN;
extern const double BALL_PLACEMENT_MARGIN;

// Invariant constants
extern const uint8_t FUZZY_TRUE;
extern const uint8_t FUZZY_FALSE;
extern const double FUZZY_MARGIN;
extern const double FUZZY_DEFAULT_CUTOFF;

// Distance constants
extern const double DISTANCE_TO_ROBOT_CLOSE;
extern const double DISTANCE_TO_ROBOT_FAR;
extern const double ROBOT_CLOSE_TO_POINT;
extern const double DISTANCE_TO_ROBOT_NEAR;
extern const double DEFENSE_AREA_AVOIDANCE_MARGIN;

// Keeper constants
extern const double DISTANCE_FROM_GOAL_CLOSE;

// GameState constants
extern const double AVOID_BALL_DISTANCE;
}  // namespace rtt::ai::stp::control_constants

#endif  // RTT_CONTROLCONSTANTS_H
