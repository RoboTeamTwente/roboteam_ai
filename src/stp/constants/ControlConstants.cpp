//
// Created by jessevw on 25.03.20.
//

#include "stp/constants/ControlConstants.h"

#include <math.h>

namespace rtt::ai::stp::control_constants {
/// Kick and chip constants
constexpr double MAX_KICK_POWER = 6.5;
constexpr double MIN_KICK_POWER = 3.0;
constexpr double MAX_POWER_KICK_DISTANCE = 8;
constexpr double MAX_POWER_CHIP_DISTANCE = 9;
constexpr double MAX_CHIP_POWER = 6.5;
constexpr double MIN_CHIP_POWER = 1.01;
// Max attempts before the force_kick_chip is set to true
constexpr double MAX_KICK_ATTEMPTS = 25;
constexpr double MAX_CHIP_ATTEMPTS = 25;

/// Robot physical constants
constexpr double ROBOT_RADIUS = 0.088;
constexpr double CENTER_TO_FRONT = 0.05;

/// Dribbler constants
// The distance from robot to ball at which the dribbler should turn on
constexpr double TURN_ON_DRIBBLER_DISTANCE = 5 * ROBOT_RADIUS;

/// Ball constants
constexpr double BALL_STILL_VEL = 0.1;
constexpr double BALL_STILL_VEL2 = BALL_STILL_VEL * BALL_STILL_VEL;
constexpr double BALL_GOT_SHOT_LIMIT = 1.3;
constexpr double BALL_IS_MOVING_SLOW_LIMIT = 0.5;
constexpr double BALL_IS_CLOSE = 0.5;
constexpr double BALL_RADIUS = 0.0215;
constexpr double HAS_KICKED_ERROR_MARGIN = 1;
constexpr double HAS_CHIPPED_ERROR_MARGIN = 0.4;
constexpr double ENEMY_CLOSE_TO_BALL_DISTANCE = 1.0;

/// RobotCommand limits
// TODO: for testing, this is set to 1.89!
constexpr double MAX_VEL_CMD = 8;
constexpr double MAX_DRIBBLER_CMD = 1;
// Angle increment per tick
constexpr double ANGLE_RATE = 0.1 * M_PI;
constexpr double MAX_VEL_WHEN_HAS_BALL = 3.0;

/// GTP Constants
// Distance margin for 'goToPos'. If the robot is within this margin, goToPos is successful
constexpr double GO_TO_POS_ERROR_MARGIN = 0.08;
// Angle margin for 'goToPos'. If the robot is within this margin, goToPos is successful
constexpr double GO_TO_POS_ANGLE_ERROR_MARGIN = 0.009 * 6.0;
// Maximum inaccuracy during ballplacement
constexpr double BALL_PLACEMENT_MARGIN = 0.15;

/// Invariant constants
constexpr uint8_t FUZZY_TRUE = 255;
constexpr uint8_t FUZZY_FALSE = 0;
constexpr double FUZZY_MARGIN = 0.1;
constexpr double FUZZY_DEFAULT_CUTOFF = 127;

/// Distance constants
constexpr double DISTANCE_TO_ROBOT_CLOSE = ROBOT_RADIUS;
constexpr double DISTANCE_TO_ROBOT_FAR = 5 * ROBOT_RADIUS;
constexpr double ROBOT_CLOSE_TO_POINT = 0.2;
constexpr double DISTANCE_TO_ROBOT_NEAR = 2.2 * ROBOT_RADIUS;
constexpr double DEFENSE_AREA_AVOIDANCE_MARGIN = 0.1;

/// Keeper constants
constexpr double DISTANCE_FROM_GOAL_CLOSE = 2 * control_constants::ROBOT_RADIUS;

/// GameState constants
constexpr double AVOID_BALL_DISTANCE = 0.8;
}  // namespace rtt::ai::stp::control_constants
