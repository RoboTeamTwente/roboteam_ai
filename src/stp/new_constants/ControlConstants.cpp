//
// Created by jessevw on 25.03.20.
//

#include <cmath>
#include "stp/new_constants/ControlConstants.h"

namespace rtt::ai::stp::control_constants {
// Kick and chip constants
constexpr double MAX_KICK_POWER = 8;
constexpr double MIN_KICK_POWER = 1.01;
constexpr double DEFAULT_KICK_POWER = 5;
constexpr double MAX_CHIP_SPEED = 8;
constexpr double MAX_POWER_KICK_DISTANCE = 8;
constexpr double MAX_POWER_CHIP_DISTANCE = 9;
constexpr double MAX_CHIP_POWER = 8;
constexpr double MIN_CHIP_POWER = 1.01;

// Ball constants
constexpr double BALL_STILL_VEL = 0.1;
constexpr double BALL_IS_MOVING_VEL = 0.5;
constexpr double BALL_RADIUS = 0.0215;
constexpr double HAS_KICKED_ERROR_MARGIN = 0.5;
constexpr double HAS_CHIPPED_ERROR_MARGIN = 0.5;

// RobotCommand limits
constexpr double MAX_VEL_CMD = 8.191;
constexpr double MAX_DRIBBLER_CMD = 31;

// HasBall margins
constexpr double HAS_BALL_ANGLE_ERROR_MARGIN = 0.2;
constexpr double HAS_BALL_DISTANCE_ERROR_MARGIN = 0.2;

// GTP Constants
constexpr double GO_TO_POS_ERROR_MARGIN = 0.02;
constexpr double GO_TO_POS_ANGLE_ERROR_MARGIN = 0.02;

// Robot physical constants
constexpr double ROBOT_RADIUS = 0.089;
constexpr double ROBOT_RADIUS_MAX = 0.091;
constexpr double FRONT_LENGTH = 0.118;
const double DRIBBLER_ANGLE_OFFSET = asin(FRONT_LENGTH / 2 / ROBOT_RADIUS);
const double CENTER_TO_FRONT = sin(DRIBBLER_ANGLE_OFFSET) * ROBOT_RADIUS;
}  // namespace rtt::ai::stp::control_constants
