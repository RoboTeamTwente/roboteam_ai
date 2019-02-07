//
// Created by mrlukasbos on 23-10-18.
//

#ifndef ROBOTEAM_AI_CONSTANTS_H
#define ROBOTEAM_AI_CONSTANTS_H

#include <QColor>
#include <ros/node_handle.h>
#include "math.h"

namespace rtt {
namespace ai {

class Constants {
private:
    bool useGrSim = false;
    // map
    std::map<std::string, double> doubles;
    std::map<std::string, bool> bools;
    std::map<std::string, int> integers;


public:
    explicit Constants() {
        ros::NodeHandle nh;
        std::string robotOutputTarget;
        nh.getParam("robot_output_target", robotOutputTarget);
        useGrSim = robotOutputTarget != "serial"; // use the grSim as default; only serial is specifically defined

        bools["SHOW_LONGEST_TICK"]              = true;
        integers["tickRate"]                    = 60;

        // defaults
        integers["DEFAULT_ROBOT_ID"]            = 1;
        doubles["DEFAULT_KICK_POWER"]           = 5.0;
        integers["DEFAULT_GENEVA_STATE"]        = 0;

        // Robotcommand limits
        doubles["MAX_VEL_CMD"]                  = 8.191;
        integers["MAX_ID_CMD"]                  = 15;
        doubles["MAX_ANGULAR_VEL_CMD"]          = 16 * M_PI;
        doubles["MAX_KICK_POWER"]               = 8.0; // TODO: CHECK

        // AI limits
        doubles["MIN_ANGLE"]                    = -M_PI;
        doubles["MAX_ANGLE"]                    = M_PI;
        doubles["MAX_VEL"]                      = 8.0;
        doubles["MAX_VEL_BALLPLACEMENT"]        = 3.0;
        doubles["MAX_ANGULAR_VELOCITY"]         = 6.0;
        integers["MAX_KICK_CYCLES"]             = 20;
        integers["MAX_GENEVA_CYCLES"]           = 20;

        // robot metrics
        doubles["ROBOT_RADIUS"]                 = 0.089;
        doubles["FRONT_LENGTH"]                 = 0.118;
        doubles["DRIBBLER_ANGLE_OFFSET"]        = asin(0.118/2/0.089); // front length / 2 / robot radius
        doubles["CENTRE_TO_FRONT"]              = sin(asin(0.118/2/0.089))*0.089; // sin(DRIBBLER_ANGLE_OFFSET) * ROBOT_RADIUS
        doubles["BALL_RADIUS"]                  = 0.0215;
        integers["GENEVA_LEFT"]                 = 0;
        integers["GENEVA_RIGHT"]                = 5;

        // gotopos luth
        doubles["GOTOPOS_LUTH_ERROR_MARGIN"]    = 0.25;

        // dribble
        doubles["MAX_BALL_RANGE"]               = 0.05;
        doubles["MAX_BALL_BOUNCE_RANGE"]        = 0.15;
        doubles["DRIBBLE_POSDIF"]               = 0.05;
        doubles["DRIBBLE_SPEED"]                = 0.8;

    }



    //dribble
    const double MAX_BALL_RANGE                 = 0.05; // Could maybe be even less? Is a LOT lower in real life, think max 0.05 m.
    const double MAX_BALL_BOUNCE_RANGE          = 0.15;
    const double DRIBBLE_POSDIF                 = 0.05;
    const float  DRIBBLE_SPEED                  = 0.8;

    //getBall
    const double COLLISION_RADIUS               = 0.18;
    const double ANGLE_SENS                     = 0.05*M_PI;
    const double MAX_GETBALL_RANGE              = 0.7;
    const int POSSES_BALL_CYCLES                = 25;
    const double GETBALL_SPEED                  = .5;
    const double GETBALL_OVERSHOOT              = .02;//m

    //GoToPos
    const double MAX_CALCULATION_TIME           = 20.0; //max time in ms
    const double standard_luth_P                = useGrSim ? 3.0 : 2.9;
    const double standard_luth_I                = useGrSim ? 0.5 : 0.4;
    const double standard_luth_D                = useGrSim ? 2.5 : 2.4;

    //Keeper
    const double KEEPER_POST_MARGIN             = 0.08;//m
    const double KEEPER_CENTREGOAL_MARGIN       = 0.3;//m
    const double KEEPER_POSDIF                  = 0.04;

    //ballkickedtoGoal
    const double BALL_TO_GOAL_MARGIN            = BALL_RADIUS;//Margin at which a ball is still detected as 'kicked at goal' next to the goalie ends, so goalie tries to save the ball.
    const double BALL_TO_GOAL_TIME              = 1.5;//seconds

    //Intercept
    const double MAX_INTERCEPT_TIME             = 2.0;//seconds. Intercept terminates  after this time.
    const double BALL_DEFLECTION_ANGLE          = 30.0/180.0*M_PI;//angle at which a ball is considered 'deflected'
    const double INTERCEPT_POSDIF               = 0.04;//m acceptable deviation

    const double DEFAULT_MAX_VEL                = 2.0;

    // BallInDefenseAreaAndStill
    const double BALL_STILL_VEL                 = 0.1;// if the ball has velocity lower than this in defense area, keeper starts getting it

    // DribbleRotate
    const double DRIBBLE_ROTATE_WAIT_TIME       = 0.2; // seconds
    const double DRIBBLE_ROTATE_MAX_SPEED       = 0.5; //rad/s

    // Ball Placement
    const double BP_MOVE_BACK_DIST              = 0.4;
    const double BP_MOVE_TOWARDS_DIST           = 0.15;

    // Avoid ball
    const double robotWeight                    = .09;
    const double minRobotDistanceForForce       = .7;
    const double ballWeight                     = .15;
    const double minBallDistanceForForce        = .7;
    const double wallWeight                     = .05;
    const double minWallDistanceForForce        = .4;

    // Settings to toggle in interface
    const bool STD_SHOW_ROLES                   = true;
    const bool STD_SHOW_TACTICS                 = false;
    const bool STD_SHOW_TACTICS_COLORS          = true;
    const bool STD_SHOW_VELOCITIES              = true;
    const bool STD_SHOW_ANGLES                  = true;
    const bool STD_SHOW_PATHS_ALL               = false;
    const bool STD_SHOW_PATHS_CURRENT           = true;
    const bool STD_SHOW_BALL_PLACEMENT_MARKER   = true;
    const bool STD_USE_REFEREE                  = true;

    // Interface drawing constants
    const int ROBOT_DRAWING_SIZE                = 8;
    const int BALL_DRAWING_SIZE                 = 5;
    const int TACTIC_COLOR_DRAWING_SIZE         = 10;
    const int WINDOW_FIELD_MARGIN               = 5;
    const int KEEPER_HELP_DRAW_SIZE             = 7;
    const int INTERCEPT_DRAW_VECTOR_SIZE        = 5;

    // Interface color constants
    const QColor FIELD_COLOR{30, 30, 30, 255};
    const QColor FIELD_LINE_COLOR = Qt::white;
    const QColor ROBOT_COLOR_BLUE { 150, 150, 255, 255 }; // Blue
    const QColor ROBOT_COLOR_YELLOW { 255, 255, 0, 255 }; // Yellow
    const QColor BALL_COLOR { 255, 120, 50, 255 }; // Orange
    const QColor TEXT_COLOR = Qt::white;
    const QColor SELECTED_ROBOT_COLOR = Qt::magenta;

    const QColor TACTIC_1 { 255, 0, 255, 255 };
    const QColor TACTIC_2 { 0, 255, 255, 255 };
    const QColor TACTIC_3 { 255, 255, 0, 255 };
    const QColor TACTIC_4 { 255, 120, 180, 255 };
    const QColor TACTIC_5 { 255, 100, 255, 255 };
    //const QColor TACTIC_COLORS[] = [TACTIC_1, TACTIC_2, TACTIC_3, TACTIC_4, TACTIC_5];

};
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
