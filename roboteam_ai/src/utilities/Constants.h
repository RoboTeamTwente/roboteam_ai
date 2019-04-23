
#ifndef ROBOTEAM_AI_CONSTANTS_H
#define ROBOTEAM_AI_CONSTANTS_H

#include <QColor>
#include <ros/node_handle.h>
#include "math.h"


namespace rtt {
namespace ai {

    typedef std::tuple<double, double, double> pidVals;

class Constants {
public:
    static void init();
    static bool GRSIM();

    /// LOGGING ///
    static bool SHOW_LONGEST_TICK()             { return false; };
    static bool SHOW_TICK_TIME_TAKEN()          { return false; };
    static bool SHOW_NUMTREE_TIME_TAKEN()       { return false; };
    static bool SHOW_NUMTREE_DEBUG_INFO()       { return false; };
    static bool SHOW_FULL_NUMTREE_DEBUG_INFO()  { return false; };


    /// ROBOT AND RELATED ///
    static double MAX_VEL_CMD()                 { return 8.191; };
    static int GENEVA_LEFT()                    { return 0; } ;     //TODO: Might be reversed, please check
    static int GENEVA_RIGHT()                   { return 5; };
    static int MAX_ID_CMD()                     { return 15; };
    static double MAX_ANGULAR_VEL_CMD()         { return 16*M_PI; };
    static double MIN_ANGLE()                   { return -M_PI; };
    static double MAX_ANGLE()                   { return M_PI; };
    static double MAX_VEL()                     { return 8.0; };
    static double MIN_VEL()                     { return 0.2; };  // Minimum velocity to make the robot move
    static double MAX_ACC_UPPER()               { return 5.0; };  // Maximum acceleration for moving in the forward direction
    static double MAX_ACC_LOWER()               { return 3.0; };  // Maximum acceleration for moving in the sideways direction
    static double MAX_VEL_BALLPLACEMENT()       { return 3.0; };
    static int DEFAULT_ROBOT_ID()               { return 1; };
    static double MAX_ANGULAR_VELOCITY()        { return 6.0; };    // Rad per second
    static double ROBOT_RADIUS()                { return 0.089;  }; // TODO: Need to test if world_state agrees with this definition of the centre of the robot
    static double ROBOT_RADIUS_MAX()            { return 0.091; };
    static double FRONT_LENGTH()                { return 0.118; };
    static double DRIBBLER_ANGLE_OFFSET()       { return asin(FRONT_LENGTH()/2/ROBOT_RADIUS()); };  // If the angle 0 is the centre of the robot, then -DRIBBLER_ANGLE_OFFSET() points to the left and DRIBBLER_ANGLE_OFFSET() to the right.
    static double CENTRE_TO_FRONT()             { return sin(DRIBBLER_ANGLE_OFFSET())*ROBOT_RADIUS(); };
    static double BALL_RADIUS()                 { return 0.0215; };
    static int TICK_RATE()                      { return 60; };
    static double CLOSE_TO_BORDER_DISTANCE()    { return 1.2 * ROBOT_RADIUS(); };
    static int GAME_ANALYSIS_TICK_RATE()        { return 30; };

    /// GENERAL SKILLS ///
    static double DEFAULT_KICK_POWER()          { return  5.0; }; // max kick power() { return  100
    static double MAX_KICK_POWER()              { return  8.0; }; //TODO: CHECK
    static double MAX_POWER_KICK_DISTANCE()     { return 9.0; };
    static int MAX_KICK_CYCLES()                { return 20; };
    static int MAX_GENEVA_CYCLES()              { return 20; };
    static int DEFAULT_GENEVA_STATE()           { return 0; };
    static double OUT_OF_FIELD_MARGIN()         { return 0.03; };
    static double MAX_BALL_BOUNCE_RANGE()       { return GRSIM() ? 0.4 : 0.15; };
    static double MAX_BALL_RANGE()              { return GRSIM() ? 0.09 : 0.04; }; // Could maybe be even less? Is a LOT lower in real life, think max 0.05 m.
    static double HAS_BALL_ANGLE()              { return 0.2; }
    static double MAX_KICK_RANGE()              { return GRSIM() ? 0.4 : 0.04; };
    static double DEFAULT_MAX_VEL()             { return 4.0; };
    static double MAX_INTERCEPT_TIME()          { return 2.0; };    // Seconds. Intercept terminates  after this time.
    static double BALL_STILL_VEL()              { return 0.1; };    // If the ball has velocity lower than this in defense area, keeper starts getting it
    static double MIN_DISTANCE_FOR_FORCE()      { return 0.5; };
    static double GOTOPOS_ERROR_MARGIN()        { return 0.1; };
    static double DEFAULT_BALLCOLLISION_RADIUS(){ return 0.27;};

    /// KEEPER ///
    static double KEEPER_POST_MARGIN()          { return 0.08; };//m
    static double KEEPER_CENTREGOAL_MARGIN()    { return 0.3; };//m
    static double KEEPER_PENALTY_LINE_MARGIN()  { return 0.04;}//m

    /// INTERFACE ///
    static int ROBOT_DRAWING_SIZE()             { return 8; };
    static int BALL_DRAWING_SIZE()              { return 5; };
    static int TACTIC_COLOR_DRAWING_SIZE()      { return 10; };
    static int WINDOW_FIELD_MARGIN()            { return 5; };

    static int KEEPER_HELP_DRAW_SIZE()          { return 7; };
    static int INTERCEPT_DRAW_VECTOR_SIZE()     { return 5; };

    static double BP_MOVE_BACK_DIST()           { return 0.4; };
    static double BP_MOVE_TOWARDS_DIST()        { return 0.15; };

    /// SETTINGS ///
    static bool STD_SHOW_ROLES()                { return true; };
    static bool STD_SHOW_TACTICS()              { return false; };
    static bool STD_SHOW_TACTICS_COLORS()       { return true; };
    static bool STD_SHOW_VELOCITIES()           { return true; };
    static bool STD_SHOW_ANGLES()               { return true; };
    static bool STD_SHOW_VORONOI()              { return false; };
    static bool STD_SHOW_PATHS_ALL()            { return false; };
    static bool STD_SHOW_PATHS_CURRENT()        { return true; };
    static bool STD_SHOW_BALL_PLACEMENT_MARKER(){ return true; };
    static bool STD_SHOW_DEBUG_VALUES()         { return true; };
    static bool STD_USE_REFEREE()               { return false; };
    static bool STD_SHOW_AVAILABLE_PASSES()     { return false; };
    static bool STD_TIMEOUT_TO_TOP()            { return false; };


    static QColor FIELD_COLOR()                 { return GRSIM() ? QColor(30 , 30 , 30 , 255) :
                                                                   QColor(50 , 0  , 0  , 255); };
    static QColor FIELD_LINE_COLOR()            { return Qt::white; };
    static QColor ROBOT_COLOR_BLUE()            { return {150, 150, 255, 255}; }; // Blue
    static QColor ROBOT_COLOR_YELLOW()          { return {255, 255, 0  , 255}; }; // Yellow
    static QColor BALL_COLOR()                  { return {255, 120, 50 , 255}; }; // Orange
    static QColor TEXT_COLOR()                  { return Qt::white; };
    static QColor SELECTED_ROBOT_COLOR()        { return Qt::magenta; };

    static std::vector<QColor> TACTIC_COLORS()  { return { {255, 0  , 255, 255},
                                                           {255, 0  , 255, 255},
                                                           {255, 255, 0  , 255},
                                                           {255, 120, 180, 255},
                                                           {255, 100, 255, 255} }; };

    // Default PID values for the gotoposses/interface
    static pidVals standardNumTreePID()         { return GRSIM() ? pidVals(3.2, 0.0, 2.0) : pidVals(2.8, 0.6,2.3); };
    static pidVals standardForcePID()           { return GRSIM() ? pidVals(1.65, 0.0, 0.0) : pidVals(2.8, 0.6,2.3); };
    static pidVals standardBasicPID()           { return GRSIM() ? pidVals(1.65, 0.0, 0.0) : pidVals(2.8, 0.6,2.3); };

    
private:
    static bool isInitialized;
    static bool robotOutputTargetGrSim; // Don't use this value. use GRSIM() instead.
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
