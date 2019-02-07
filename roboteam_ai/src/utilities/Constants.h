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
    static bool useGrSim;
    static std::map<std::string, double> doubles;
    static std::map<std::string, bool> bools;
    static std::map<std::string, int> integers;
    static std::map<std::string, QColor> colors;

public:
    explicit Constants();
    static double getDouble(const std::string &name);
    static bool getBool(const std::string &name);
    static QColor getColor(const std::string &name);
    static int getInt(const std::string &name);

    // this is an initializer class that will act as a constructor for this static class
    // This is because static classes usually don't have a constructor
    static class _init {
    public:
        _init() {
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
            doubles["DEFAULT_MAX_VEL"]              = 2.0;

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

            // dribble
            doubles["MAX_BALL_RANGE"]               = 0.05; // Could maybe be even less? Is a LOT lower in real life, think max 0.05 m.
            doubles["MAX_BALL_BOUNCE_RANGE"]        = 0.15; // (m)
            doubles["DRIBBLE_POSDIF"]               = 0.05;
            doubles["DRIBBLE_SPEED"]                = 0.8; // (m/s)

            // getBall
            doubles["COLLISION_RADIUS"]             = 0.18; // (m)
            doubles["ANGLE_SENS"]                   = 0.05 * M_PI;
            doubles["MAX_GETBALL_RANGE"]            = 0.7;
            integers["POSSES_BALL_CYCLES"]          = 25;
            doubles["GETBALL_SPEED"]                = .5; // (m/s)
            doubles["GETBALL_OVERSHOOT"]            = 0.02;

            // GoToPos
            doubles["MAX_CALCULATION_TIME"]         = 20.0;
            doubles["standard_luth_P"]              = useGrSim ? 3.0 : 2.9;
            doubles["standard_luth_I"]              = useGrSim ? 0.5 : 0.4;
            doubles["standard_luth_D"]              = useGrSim ? 2.5 : 2.4;
            doubles["GOTOPOS_LUTH_ERROR_MARGIN"]    = 0.25;

            // Keeper
            doubles["KEEPER_POST_MARGIN"]           = 0.08;
            doubles["KEEPER_CENTREGOAL_MARGIN"]     = 0.3;
            doubles["KEEPER_POSDIF"]                = 0.04;

            // ballkickedtoGoal
            // Margin at which a ball is still detected as 'kicked at goal' next to the goalie ends, so goalie tries to save the ball.
            doubles["BALL_TO_GOAL_MARGIN"]          = 0.0215;
            doubles["BALL_TO_GOAL_TIME"]            = 1.5;

            //Intercept
            doubles["MAX_INTERCEPT_TIME"]           = 2.0;
            doubles["BALL_DEFLECTION_ANGLE"]        = 30.0/180.0*M_PI; // (rad)
            doubles["INTERCEPT_POSDIF"]             = 0.04;

            // BallInDefenseAreaAndStill
            doubles["BALL_STILL_VEL"]               = 0.1;

            // DribbleRotate
            doubles["DRIBBLE_ROTATE_WAIT_TIME"]     = 0.2;
            doubles["DRIBBLE_ROTATE_MAX_SPEED"]     = 0.5;

            // Ball placement
            doubles["BP_MOVE_BACK_DIST"]            = 0.4;
            doubles["BP_MOVE_TOWARDS_DIST"]         = 0.15;

            // AvoidBall
            doubles["robotWeight"]                  = 0.09;
            doubles["minRobotDistanceForForce"]     = 0.7;
            doubles["ballWeight"]                   = 0.15;
            doubles["minBallDistanceForForce"]      = 0.7;
            doubles["wallWeight"]                   = .05;
            doubles["minWallDistanceForForce"]      = .4;

            // interface - default checkbox values
            bools["STD_SHOW_ROLES"]                 = true;
            bools["STD_SHOW_TACTICS"]               = false;
            bools["STD_SHOW_TACTICS_COLORS"]        = true;
            bools["STD_SHOW_VELOCITIES"]            = true;
            bools["STD_SHOW_ANGLES"]                = true;
            bools["STD_SHOW_PATHS_ALL"]             = false;
            bools["STD_SHOW_PATHS_CURRENT"]         = true;
            bools["STD_SHOW_BALL_PLACEMENT_MARKER"] = true;
            bools["STD_USE_REFEREE"]                = true;

            // interface - drawing sizes on screen
            integers["ROBOT_DRAWING_SIZE"]          = 8;  // px
            integers["BALL_DRAWING_SIZE"]           = 5;  // px
            integers["TACTIC_COLOR_DRAWING_SIZE"]   = 10; // px
            integers["WINDOW_FIELD_MARGIN"]         = 5;  // px
            integers["KEEPER_HELP_DRAW_SIZE"]       = 7;  // px
            integers["INTERCEPT_DRAW_VECTOR_SIZE"]  = 5;  // px

            colors["FIELD_COLOR"]                   = {30, 30, 30, 255}; // rgba - gray
            colors["FIELD_LINE_COLOR"]              = Qt::white;
            colors["ROBOT_COLOR_BLUE"]              = { 150, 150, 255, 255 }; // Blue
            colors["ROBOT_COLOR_YELLOW"]            = { 255, 255, 0, 255 }; // Yellow
            colors["BALL_COLOR"]                    = { 255, 120, 50, 255 }; // Orange
            colors["TEXT_COLOR"]                    = Qt::white;
            colors["SELECTED_ROBOT_COLOR"]          = Qt::magenta;

            colors["TACTIC_1"]                      = { 255, 0, 255, 255 };
            colors["TACTIC_2"]                      = { 0, 255, 255, 255 };
            colors["TACTIC_3"]                      = { 255, 255, 0, 255 };
            colors["TACTIC_4"]                      = { 255, 120, 180, 255 };
            colors["TACTIC_5"]                      = { 255, 100, 255, 255 };
            //const QColor TACTIC_COLORS[] = [TACTIC_1, TACTIC_2, TACTIC_3, TACTIC_4, TACTIC_5];
        }
    } _initializer;
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
