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

// Dribbler constants
const double TURN_ON_DRIBBLER_DISTANCE = 3 * ROBOT_RADIUS;

// Ball constants
constexpr double BALL_STILL_VEL = 0.1;
constexpr double BALL_IS_MOVING_VEL = 0.5;
constexpr double BALL_GOT_SHOT_LIMIT = 1.3;
constexpr double BALL_IS_MOVING_SLOW_LIMIT = 0.5;
constexpr double BALL_IS_CLOSE = 0.5;
constexpr double BALL_RADIUS = 0.0215;
constexpr double HAS_KICKED_ERROR_MARGIN = 0.4;
constexpr double HAS_CHIPPED_ERROR_MARGIN = 0.4;
constexpr double ENEMY_CLOSE_TO_BALL_DISTANCE = 1.0;

// RobotCommand limits
constexpr double MAX_VEL_CMD = 8.191;
constexpr double MAX_DRIBBLER_CMD = 31;

// HasBall margins
constexpr double HAS_BALL_ANGLE_ERROR_MARGIN = 0.106;
constexpr double HAS_BALL_DISTANCE_ERROR_MARGIN = 0.206;

// GTP Constants
constexpr double GO_TO_POS_ERROR_MARGIN = 0.01;
constexpr double GO_TO_POS_ANGLE_ERROR_MARGIN = 0.01;

// Robot physical constants
constexpr double ROBOT_RADIUS = 0.089;
constexpr double ROBOT_RADIUS_MAX = 0.091;
constexpr double FRONT_LENGTH = 0.118;
const double DRIBBLER_ANGLE_OFFSET = asin(FRONT_LENGTH / 2 / ROBOT_RADIUS);
const double CENTER_TO_FRONT = sin(DRIBBLER_ANGLE_OFFSET) * ROBOT_RADIUS;

// Invariant constants
constexpr double FUZZY_TRUE = 255;
constexpr double FUZZY_FALSE = 0;
constexpr double FUZZY_MARGIN = 0.1;
constexpr double FUZZY_DEFAULT_CUTOFF = 127;

// Distance constants
const double DISTANCE_TO_ROBOT_CLOSE = ROBOT_RADIUS; // TODO: TUNE
const double DISTANCE_TO_ROBOT_FAR = 5 * ROBOT_RADIUS; // TODO: TUNE
constexpr double ROBOT_CLOSE_TO_POINT = 0.2;

// Keeper constants
const double DISTANCE_FROM_GOAL_CLOSE = 2 * control_constants::ROBOT_RADIUS;

// GameState constants
const double AVOID_BALL_DISTANCE = 0.55;
}  // namespace rtt::ai::stp::control_constants
