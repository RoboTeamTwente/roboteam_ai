//
// Created by mrlukasbos on 23-10-18.
//

#ifndef ROBOTEAM_AI_CONSTANTS_H
#define ROBOTEAM_AI_CONSTANTS_H
//TODO: add units to the below things, check with control/robothub.

#include <QColor>
#include "math.h"

namespace rtt {
namespace ai {
namespace constants {


const bool SHOW_LONGEST_TICK = true;
const double GOTOPOS_LUTH_ERROR_MARGIN = 0.25;

// Max values we can send through robothub
const double MAX_VEL_CMD=8.191;
const int GENEVA_LEFT=0;//TODO: Might be reversed, please check
const int GENEVA_RIGHT=5;
const int MAX_ID_CMD=15;
const double MAX_ANGULAR_VEL_CMD=16*M_PI;
const double MIN_ANGLE = -M_PI;
const double MAX_ANGLE = M_PI;

// Limits as defined in AI itself
const double MAX_VEL=8.0;
//Other/multiple usage
const int DEFAULT_ROBOT_ID = 1;
const double MAX_ANGULAR_VELOCITY = 6.0; // rad per second??
const double ROBOT_RADIUS=0.089; // TODO: Need to test if world_state agrees with this definition of the centre of the robot
const double FRONT_LENGTH=0.118; // length of the front (flat) part of the robot
const double DRIBBLER_ANGLE_OFFSET=asin(FRONT_LENGTH/2/ROBOT_RADIUS); // if the angle 0 is the centre of the robot, then -DRIBBLER_ANGLE_OFFSET points to the left and DRIBBLER_ANGLE_OFFSET to the right.
const double CENTRE_TO_FRONT=sin(DRIBBLER_ANGLE_OFFSET)*ROBOT_RADIUS;
const double BALL_RADIUS=0.0215;

const int tickRate=60 ;// Rate at which we tick our behavior Trees

//skills
const double DEFAULT_KICK_POWER = 5.0; // max kick power = 100
const double MAX_KICK_POWER = 8.0; //TODO: CHECK
const int MAX_KICK_CYCLES = 20;
const int MAX_GENEVA_CYCLES = 20;
const int DEFAULT_GENEVA_STATE = 0;

//dribble
const double MAX_BALL_RANGE=0.05; // Could maybe be even less? Is a LOT lower in real life, think max 0.05 m.
const double MAX_BALL_BOUNCE_RANGE=0.3;
const double DRIBBLE_POSDIF=0.05;
const float  DRIBBLE_SPEED=0.8;
//getBallcc
const double COLLISION_RADIUS=0.18;
const double ANGLE_SENS=0.05*M_PI;
const double MAX_GETBALL_RANGE=0.7;
const int POSSES_BALL_CYCLES=100;
const double GETBALL_SPEED=.5;
const double GETBALL_OVERSHOOT=.05;//m

//GoToPos
const double MAX_CALCULATION_TIME=20.0; //max time in ms

//Keeper
const double KEEPER_POST_MARGIN=0.08;//m
const double KEEPER_CENTREGOAL_MARGIN=0.3;//m
const double KEEPER_POSDIF=0.04;

//ballkickedtoGoal
const double BALL_TO_GOAL_MARGIN=BALL_RADIUS;//Margin at which a ball is still detected as 'kicked at goal' next to the goalie ends, so goalie tries to save the ball.
const double BALL_TO_GOAL_TIME=3;//seconds

//Intercept
const double MAX_INTERCEPT_TIME=2.0;//seconds. Intercept terminates  after this time.
const double BALL_DEFLECTION_ANGLE=30.0/180.0*M_PI;//angle at which a ball is considered 'deflected'
const double INTERCEPT_POSDIF=0.04;//m acceptable deviation

const double DEFAULT_MAX_VEL=2.0;
// BallInDefenseAreaAndStill
const double BALL_STILL_VEL=0.1;// if the ball has velocity lower than this in defense area, keeper starts getting it

const double DRIBBLE_ROTATE_WAIT_TIME=0.2; // seconds
const double DRIBBLE_ROTATE_MAX_SPEED=0.5; //rad/s
// Interface
const int ROBOT_DRAWING_SIZE = 8;
const int BALL_DRAWING_SIZE = 5;
const int TACTIC_COLOR_DRAWING_SIZE = 10;
const int WINDOW_FIELD_MARGIN = 5;

const int KEEPER_HELP_DRAW_SIZE=7;
const int INTERCEPT_DRAW_VECTOR_SIZE=5;

const double BP_MOVE_BACK_DIST=0.4;
const double BP_MOVE_TOWARDS_DIST=0.15;

// Settings
const bool STD_SHOW_ROLES = true;
const bool STD_SHOW_TACTICS = false;
const bool STD_SHOW_TACTICS_COLORS = true;
const bool STD_SHOW_VELOCITIES = true;
const bool STD_SHOW_ANGLES = true;
const bool STD_SHOW_VORONOI = false;
const bool STD_SHOW_PATHS_ALL = false;
const bool STD_SHOW_PATHS_CURRENT = true;
const bool STD_SHOW_BALL_PLACEMENT_MARKER = true;

const QColor FIELD_COLOR{30, 30, 30, 255};
const QColor FIELD_LINE_COLOR = Qt::white;
const QColor ROBOT_US_COLOR { 150, 150, 255, 255 }; // Blue
const QColor ROBOT_THEM_COLOR { 255, 255, 0, 255 }; // Yellow
const QColor BALL_COLOR { 255, 120, 50, 255 }; // Orange
const QColor TEXT_COLOR = Qt::white;
const QColor SELECTED_ROBOT_COLOR = Qt::magenta;

const QColor TACTIC_1 { 255, 0, 255, 255 };
const QColor TACTIC_2 { 0, 255, 255, 255 };
const QColor TACTIC_3 { 255, 255, 0, 255 };
const QColor TACTIC_4 { 255, 120, 180, 255 };
const QColor TACTIC_5 { 255, 100, 255, 255 };
const QColor TACTIC_COLORS[] = {TACTIC_1, TACTIC_2, TACTIC_3, TACTIC_4, TACTIC_5};

const double standard_luth_P = 3.0;
const double standard_luth_I = 0.5;
const double standard_luth_D = 2.5;
} // constants
} // ai
} // rtt


enum class RefGameState {
    // Ref states as dictated by RoboCup SSL
    HALT = 0,
    STOP = 1,
    NORMAL_START = 2,
    FORCED_START = 3,
    PREPARE_KICKOFF_US = 4,
    PREPARE_KICKOFF_THEM = 5,
    PREPARE_PENALTY_US = 6,
    PREPARE_PENALTY_THEM = 7,
    DIRECT_FREE_US = 8,
    DIRECT_FREE_THEM = 9,
    INDIRECT_FREE_US = 10,
    INDIRECT_FREE_THEM = 11,
    TIMEOUT_US = 12,
    TIMEOUT_THEM = 13,
    GOAL_US = 14,
    GOAL_THEM = 15,
    BALL_PLACEMENT_US = 16,
    BALL_PLACEMENT_THEM = 17,

    // Custom extended refstates
    // These numbers will never be called from the referee immediately, they can only be used as follow-up commands
    DO_KICKOFF = 18,
    DEFEND_KICKOFF = 19,
    DO_PENALTY = 20,
    DEFEND_PENALTY = 21,
    UNDEFINED = -1
};

#endif //ROBOTEAM_AI_CONSTANTS_H
