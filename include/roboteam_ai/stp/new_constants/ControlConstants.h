//
// Created by jessevw on 25.03.20.
//

#ifndef RTT_CONTROLCONSTANTS_H
#define RTT_CONTROLCONSTANTS_H

namespace rtt::ai::stp::control_constants {
// Kick and Chip constants
extern const double MAX_CHIP_SPEED;
extern const double MAX_KICK_POWER;
extern const double MIN_KICK_POWER;
extern const double MAX_CHIP_POWER;
extern const double MIN_CHIP_POWER;
extern const double DEFAULT_KICK_POWER;
extern const double MAX_POWER_KICK_DISTANCE;
extern const double MAX_POWER_CHIP_DISTANCE;

// Dribbler constants
extern const double TURN_ON_DRIBBLER_DISTANCE;

// Team constants TODO: Maybe this should be in a different constants file
inline constexpr size_t MAX_ROBOT_COUNT = 11;

// Ball constants
extern const double BALL_STILL_VEL;
extern const double BALL_IS_MOVING_VEL;
extern const double BALL_IS_MOVING_FAST_LIMIT;
extern const double BALL_RADIUS;
extern const double HAS_KICKED_ERROR_MARGIN;
extern const double HAS_CHIPPED_ERROR_MARGIN;

// RobotCommand limits
extern const double MAX_VEL_CMD;
extern const double MAX_DRIBBLER_CMD;

// HasBall margins
extern const double HAS_BALL_ANGLE_ERROR_MARGIN;
extern const double HAS_BALL_DISTANCE_ERROR_MARGIN;

// GTP Constants
extern const double GO_TO_POS_ERROR_MARGIN;
extern const double GO_TO_POS_ANGLE_ERROR_MARGIN;

// Robot physical constants
extern const double ROBOT_RADIUS;
extern const double ROBOT_RADIUS_MAX;
extern const double FRONT_LENGTH;
extern const double DRIBBLER_ANGLE_OFFSET;
extern const double CENTER_TO_FRONT;
}  // namespace rtt::ai::stp::control_constants

#endif  // RTT_CONTROLCONSTANTS_H
