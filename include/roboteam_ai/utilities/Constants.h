
#ifndef ROBOTEAM_AI_CONSTANTS_H
#define ROBOTEAM_AI_CONSTANTS_H

#include <QColor>
#include <map>
#include <vector>
#include "RuleSet.h"
#include "math.h"

namespace rtt::ai {

typedef std::tuple<double, double, double> pidVals;

    class Constants {
   public:
    static void init();
    static bool GRSIM();
    static void OVERWRITE_GRSIM(bool grsim);
    static bool FEEDBACK_ENABLED();

    static constexpr size_t ROBOT_COUNT() {return 11; };

    /// TICK RATE ///
    static constexpr int GAME_ANALYSIS_TICK_RATE() { return 5; };
    static constexpr int TICK_RATE() { return 60; };

    /// LOGGING ///
    static bool SHOW_LONGEST_TICK();
    static bool SHOW_TICK_TIME_TAKEN();

    static bool SHOW_COACH_TIME_TAKEN();

    static bool SHOW_NUMTREE_TIME_TAKEN();
    static bool SHOW_NUMTREE_DEBUG_INFO();
    static bool SHOW_FULL_NUMTREE_DEBUG_INFO();

    static bool SHOW_BALL_HANDLE_DEBUG_INFO();
    static bool SHOW_FULL_BALL_HANDLE_DEBUG_INFO();

    static std::vector<RuleSet> ruleSets();

    /// ROBOT COMMANDS ///
    static double MAX_VEL_CMD();
    static int MAX_ID_CMD();
    static double MAX_ANGULAR_VEL_CMD();
    static double MIN_ANGLE();
    static double MAX_ANGLE();
    static double MAX_ANGULAR_VELOCITY();  // Rad per second
    static int MAX_DRIBBLER_CMD();

    /// ACCELERATION LIMITERS ///
    static double MIN_VEL();        // Minimum velocity to make the robot move
    static double MAX_ACC_UPPER();  // Maximum acceleration for moving in the forward direction
    static double MAX_ACC_LOWER();  // Maximum acceleration for moving in the sideways direction
    static double MAX_DEC_UPPER();  // Maximum deceleration for moving in the forward direction
    static double MAX_DEC_LOWER();  // Maximum deceleration for moving in the sideways direction

    /// ROBOT SPECS ///
    static constexpr double ROBOT_RADIUS() { return 0.089; };
    static constexpr double ROBOT_RADIUS_MAX() { return 0.091; };
    static constexpr double BALL_RADIUS() { return 0.0215; };

    static double FRONT_LENGTH();
    static double DRIBBLER_ANGLE_OFFSET();  // If the angle 0 is the centre of the robot, then -DRIBBLER_ANGLE_OFFSET() points to the left and DRIBBLER_ANGLE_OFFSET() to the right.
    static double CENTRE_TO_FRONT();
    static double CLOSE_TO_BORDER_DISTANCE();

    /// REF STATES ///
    static constexpr double MAX_VEL() { return 8.0; };
    static constexpr double MAX_STOP_STATE_VEL() { return 1.5; };
    static constexpr double MAX_VEL_BALLPLACEMENT() { return 3.0; };
    static int DEFAULT_KEEPER_ID();

    /// GENERAL SKILLS ///
    static double DEFAULT_KICK_POWER();
    static double MAX_KICK_POWER();      // TODO: TUNE MAX KICK POWER
    static double MIN_KICK_POWER();      // TODO: TUNE MIN KICK POWER
    static double MAX_POWER_KICK_DISTANCE(); // TODO: TUNE MAX KICK DISTANCE
    static double MAX_CHIP_POWER();      // TODO: TUNE MAX CHIP POWER
    static double MIN_CHIP_POWER();      // TODO: TUNE MIN CHIP POWER
    static double MAX_POWER_CHIP_DISTANCE(); // TODO: TUNE MAX CHIP DISTANCE
    static double OUT_OF_FIELD_MARGIN();
    static double MAX_BALL_BOUNCE_RANGE();
    static double MAX_BALL_RANGE();  // Could maybe be even less? Is a LOT lower in real life, think max 0.05 m.
    static double HAS_BALL_ANGLE();
    static double MAX_KICK_RANGE();
    static double MAX_PASS_DISTANCE();
    static bool REFLECT_KICK();

    static double MAX_INTERCEPT_TIME();  // Seconds. Intercept terminates  after this time.
    static double MAX_RECEIVE_TIME();
    static double BALL_STILL_VEL();  // If the ball has velocity lower than this in defense area, keeper starts getting it
    static double GOTOPOS_ERROR_MARGIN();
    static double GOTOPOS_ANGLE_ERROR_MARGIN();
    static double DEFAULT_BALLCOLLISION_RADIUS();

    /// KEEPER ///
    static double KEEPER_POST_MARGIN();          // m
    static double KEEPER_CENTREGOAL_MARGIN();    // m
    static double KEEPER_PENALTY_LINE_MARGIN();  // m

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
    static bool STD_SHOW_ROBOT_INVALIDS();
    static bool STD_SHOW_BALL_PLACEMENT_MARKER();
    static bool STD_SHOW_DEBUG_VALUES();
    static bool STD_USE_REFEREE();
    static bool STD_TIMEOUT_TO_TOP();

    static std::map<int, bool> ROBOTS_WITH_WORKING_DRIBBLER();
    static std::map<int, bool> ROBOTS_WITH_WORKING_BALL_SENSOR();

    static bool ROBOT_HAS_WORKING_DRIBBLER(int id);
    static bool ROBOT_HAS_WORKING_BALL_SENSOR(int id);

    static QColor FIELD_COLOR();
    static QColor FIELD_LINE_COLOR();
    static QColor ROBOT_COLOR_BLUE();    // Blue
    static QColor ROBOT_COLOR_YELLOW();  // Yellow
    static QColor BALL_COLOR();          // Orange
    static QColor TEXT_COLOR();
    static QColor SELECTED_ROBOT_COLOR();

    static std::vector<QColor> TACTIC_COLORS();

    // Default PID values for the gotoposses/interface
    static pidVals standardNumTreePID();
    static pidVals standardReceivePID();
    static pidVals standardInterceptPID();
    static pidVals standardKeeperPID();
    static pidVals standardKeeperInterceptPID();

   private:
    static bool isInitialized;
    static bool robotOutputTargetGrSim;  // Don't use this value. use GRSIM() instead.
};

}  // namespace rtt::ai

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
    // Custom refstates; these are
    PREPARE_SHOOTOUT_US = 22,
    PREPARE_SHOOTOUT_THEM = 23,
    // Extended custom refstates: extension upon custom refstates
    DO_SHOOTOUT = 24,
    DEFEND_SHOOTOUT = 25,

    UNDEFINED = -1
};

#endif  // ROBOTEAM_AI_CONSTANTS_H
