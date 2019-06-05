
#ifndef ROBOTEAM_AI_CONSTANTS_H
#define ROBOTEAM_AI_CONSTANTS_H

#include <QColor>
#include <ros/node_handle.h>
#include "math.h"
#include "RuleSet.h"

namespace rtt {
namespace ai {

typedef std::tuple<double, double, double> pidVals;

class Constants {

    public:
        static void init();
        static bool GRSIM();

        /// LOGGING ///
        static bool SHOW_LONGEST_TICK();
        static bool SHOW_TICK_TIME_TAKEN();

        static bool SHOW_NUMTREE_TIME_TAKEN();
        static bool SHOW_COACH_TIME_TAKEN();

        static bool SHOW_NUMTREE_DEBUG_INFO();
        static bool SHOW_FULL_NUMTREE_DEBUG_INFO();
        static bool SHOW_BALL_HANDLE_DEBUG_INFO();
        static bool SHOW_FULL_BALL_HANDLE_DEBUG_INFO();

        // Basic rulesets for rule compliance
        static std::vector<RuleSet> ruleSets();

        /// ROBOT AND RELATED ///
        static double MAX_VEL_CMD();
        static int MAX_ID_CMD();
        static double MAX_ANGULAR_VEL_CMD();
        static double MIN_ANGLE();
        static double MAX_ANGLE();

        // max velocities for refstates
        static double MAX_VEL();
        static double MAX_STOP_STATE_VEL();

        static double MIN_VEL();  // Minimum velocity to make the robot move
        static double MAX_ACC_UPPER();  // Maximum acceleration for moving in the forward direction
        static double MAX_ACC_LOWER();  // Maximum acceleration for moving in the sideways direction
        static double MAX_DEC_UPPER();  // Maximum deceleration for moving in the forward direction
        static double MAX_DEC_LOWER();  // Maximum deceleration for moving in the sideways direction

        static double MAX_VEL_BALLPLACEMENT();
        static double MAX_ANGULAR_VELOCITY();    // Rad per second
        static double ROBOT_RADIUS(); // TODO: Need to test if world_state agrees with this definition of the centre of the robot
        static double ROBOT_RADIUS_MAX();
        static double FRONT_LENGTH();
        static double DRIBBLER_ANGLE_OFFSET();  // If the angle 0 is the centre of the robot, then -DRIBBLER_ANGLE_OFFSET() points to the left and DRIBBLER_ANGLE_OFFSET() to the right.
        static double CENTRE_TO_FRONT();
        static double BALL_RADIUS();
        static int TICK_RATE();
        static double CLOSE_TO_BORDER_DISTANCE();
        static int GAME_ANALYSIS_TICK_RATE();

        /// GENERAL SKILLS ///
        static double DEFAULT_KICK_POWER(); // max kick power() { return  100
        static double MAX_KICK_POWER(); //TODO: CHECK
        static double MAX_POWER_KICK_DISTANCE();
        static int MAX_KICK_CYCLES();
        static double OUT_OF_FIELD_MARGIN();
        static double MAX_BALL_BOUNCE_RANGE();
        static double MAX_BALL_RANGE(); // Could maybe be even less? Is a LOT lower in real life, think max 0.05 m.
        static double HAS_BALL_ANGLE();
        static double MAX_KICK_RANGE();

        static double MAX_INTERCEPT_TIME();    // Seconds. Intercept terminates  after this time.
        static double BALL_STILL_VEL();    // If the ball has velocity lower than this in defense area, keeper starts getting it
        static double MIN_DISTANCE_FOR_FORCE();
        static double GOTOPOS_ERROR_MARGIN();
        static double GOTOPOS_ANGLE_ERROR_MARGIN();
        static double DEFAULT_BALLCOLLISION_RADIUS();

        /// KEEPER ///
        static double KEEPER_POST_MARGIN();//m
        static double KEEPER_CENTREGOAL_MARGIN();//m
        static double KEEPER_PENALTY_LINE_MARGIN();//m

        /// INTERFACE ///
        static int ROBOT_DRAWING_SIZE();
        static int BALL_DRAWING_SIZE();
        static int TACTIC_COLOR_DRAWING_SIZE();
        static int WINDOW_FIELD_MARGIN();

        static int KEEPER_HELP_DRAW_SIZE();
        static int INTERCEPT_DRAW_VECTOR_SIZE();

        static double BP_MOVE_BACK_DIST();
        static double BP_MOVE_TOWARDS_DIST();

        /// SETTINGS ///
        static bool STD_SHOW_ROLES();
        static bool STD_SHOW_TACTICS();
        static bool STD_SHOW_TACTICS_COLORS();
        static bool STD_SHOW_VELOCITIES();
        static bool STD_SHOW_ANGLES();
        static bool STD_SHOW_PATHS_ALL();
        static bool STD_SHOW_PATHS_CURRENT();
        static bool STD_SHOW_BALL_PLACEMENT_MARKER();
        static bool STD_SHOW_DEBUG_VALUES();
        static bool STD_USE_REFEREE();
        static bool STD_TIMEOUT_TO_TOP();

        static std::map<int, bool> ROBOTS_WITH_WORKING_GENEVA();
        static std::map<int, bool> ROBOTS_WITH_WORKING_DRIBBLER();

        static bool ROBOT_HAS_WORKING_GENEVA(int id);
        static bool ROBOT_HAS_WORKING_DRIBBLER(int id);

        static QColor FIELD_COLOR();
        static QColor FIELD_LINE_COLOR();
        static QColor ROBOT_COLOR_BLUE(); // Blue
        static QColor ROBOT_COLOR_YELLOW(); // Yellow
        static QColor BALL_COLOR(); // Orange
        static QColor TEXT_COLOR();
        static QColor SELECTED_ROBOT_COLOR();

        static std::vector<QColor> TACTIC_COLORS();

        // Default PID values for the gotoposses/interface
        static pidVals standardNumTreePID();
        static pidVals standardForcePID();
        static pidVals standardBasicPID();
        static pidVals standardShotControllerPID();

    private:
        static bool isInitialized;
        static bool robotOutputTargetGrSim; // Don't use this value. use GRSIM() instead.
};

} // ai
} // rtt


enum class RefCommand {
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
        UNDEFINED = - 1
};

#endif //ROBOTEAM_AI_CONSTANTS_H
