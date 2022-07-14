//
// Created by mrlukasbos on 8-2-19.
//

#include <assert.h>
#include <roboteam_utils/Print.h>

#include "utilities/Settings.h"
#include "utilities/Constants.h"

// TODO: Clean this up and remove unneeded variables

namespace rtt::ai {

// static initializers
bool Constants::isInitialized = false;
bool Constants::robotOutputTargetGrSim = true;

void Constants::init() {
    std::string target = robotOutputTargetGrSim ? "GRSIM" : "SERIAL";
    RTT_INFO("robot_output_target is set to ", target)
    isInitialized = true;
}

bool Constants::GRSIM() {
    if (!isInitialized) {
        RTT_ERROR("You use a value dependent on an unkown environment! This may result in unexepected behaviour")
        assert(false);
    }
    return robotOutputTargetGrSim;
}

void Constants::OVERWRITE_GRSIM(bool grsim) {
    RTT_WARNING("Do not overwrite GRSIM() if you do not know what you are doing!")
    robotOutputTargetGrSim = grsim;
}

double Constants::PENALTY_DISTANCE_BEHIND_BALL() {
    // The minimum is 1 meter, but do 1.5 to be sure
    return 1.5;
}

/// Set to a valid Id to make that robot keeper. Otherwise, keeper will be first distributed based on cost.
int Constants::DEFAULT_KEEPER_ID() { return -1; }

bool Constants::FEEDBACK_ENABLED() { return true; }

double Constants::FRONT_LENGTH() { return 0.118; }

double Constants::MAX_ANGULAR_VELOCITY() { return 6.0; }

bool Constants::SHOW_LONGEST_TICK() { return true; }

bool Constants::SHOW_TICK_TIME_TAKEN() { return true; }

bool Constants::SHOW_NUMTREE_TIME_TAKEN() { return false; }

bool Constants::SHOW_COACH_TIME_TAKEN() { return false; }

bool Constants::SHOW_NUMTREE_DEBUG_INFO() { return false; }
bool Constants::SHOW_FULL_NUMTREE_DEBUG_INFO() { return false; }
bool Constants::SHOW_BALL_HANDLE_DEBUG_INFO() { return false; }
bool Constants::SHOW_FULL_BALL_HANDLE_DEBUG_INFO() { return false; }

double Constants::MAX_VEL_CMD() { return 4.0; }

int Constants::MAX_ID_CMD() { return 15; }

double Constants::MAX_ANGULAR_VEL_CMD() { return 16 * M_PI; }

double Constants::MIN_ANGLE() { return -M_PI; }

double Constants::MAX_ANGLE() { return M_PI; }

int Constants::MAX_DRIBBLER_CMD() { return 1; }

double Constants::MIN_VEL() { return 0.2; }

double Constants::MAX_ACC_UPPER() { return 3.0; }

double Constants::MAX_ACC_LOWER() { return 3.0; }

double Constants::MAX_DEC_UPPER() { return MAX_ACC_UPPER() * 1.2; }  // magic number

double Constants::MAX_DEC_LOWER() { return MAX_ACC_LOWER() * 1.2; }  // magic number

double Constants::DRIBBLER_ANGLE_OFFSET() { return asin(FRONT_LENGTH() / 2 / ROBOT_RADIUS()); }

double Constants::CENTRE_TO_FRONT() { return sin(DRIBBLER_ANGLE_OFFSET()) * ROBOT_RADIUS(); }

double Constants::CLOSE_TO_BORDER_DISTANCE() { return 1.2 * ROBOT_RADIUS(); }

double Constants::DEFAULT_KICK_POWER() { return 5.0; }

double Constants::MAX_POWER_KICK_DISTANCE() { return 9.0; }

double Constants::MAX_KICK_POWER() { return 8.0; }

double Constants::MIN_KICK_POWER() { return 1.01; }

double Constants::MAX_POWER_CHIP_DISTANCE() { return 9.0; }

double Constants::MAX_CHIP_POWER() { return 8.0; }

double Constants::MIN_CHIP_POWER() { return 1.01; }

double Constants::MAX_PASS_DISTANCE() { return 6.0; }

bool Constants::REFLECT_KICK() { return true; }

double Constants::OUT_OF_FIELD_MARGIN() { return 0.03; }

double Constants::MAX_BALL_BOUNCE_RANGE() { return GRSIM() ? 0.4 : 0.15; }

double Constants::MAX_KICK_RANGE() { return 0.05; }

double Constants::MAX_INTERCEPT_TIME() { return 3.0; }

double Constants::MAX_RECEIVE_TIME() { return 1.0; }

double Constants::BALL_STILL_VEL() { return 0.1; }

double Constants::GOTOPOS_ERROR_MARGIN() { return 0.02; }  // was 0.03 before STP

double Constants::GOTOPOS_ANGLE_ERROR_MARGIN() { return 0.03; }

double Constants::DEFAULT_BALLCOLLISION_RADIUS() { return 0.27; }

double Constants::KEEPER_POST_MARGIN() { return 0.08; }

double Constants::KEEPER_CENTREGOAL_MARGIN() { return 0.2; }

double Constants::KEEPER_PENALTY_LINE_MARGIN() { return 0.06; }

int Constants::ROBOT_DRAWING_SIZE() { return 6; }

int Constants::BALL_DRAWING_SIZE() { return 4; }

int Constants::TACTIC_COLOR_DRAWING_SIZE() { return 15; }

int Constants::WINDOW_FIELD_MARGIN() { return 5; }

int Constants::KEEPER_HELP_DRAW_SIZE() { return 7; }

int Constants::INTERCEPT_DRAW_VECTOR_SIZE() { return 5; }

double Constants::BP_MOVE_BACK_DIST() { return 0.4; }

double Constants::BP_MOVE_TOWARDS_DIST() { return 0.15; }

bool Constants::STD_SHOW_ROLES() { return true; }

bool Constants::STD_SHOW_TACTICS() { return false; }

bool Constants::STD_SHOW_TACTICS_COLORS() { return true; }

bool Constants::STD_SHOW_VELOCITIES() { return true; }

bool Constants::STD_SHOW_ANGLES() { return true; }

bool Constants::STD_SHOW_ROBOT_INVALIDS() { return true; }

bool Constants::STD_SHOW_BALL_PLACEMENT_MARKER() { return true; }

bool Constants::STD_SHOW_DEBUG_VALUES() { return true; }

bool Constants::STD_USE_REFEREE() { return true; }

bool Constants::STD_TIMEOUT_TO_TOP() { return false; }

// The max distance the ball can be from the robot for the robot to have the ball
double Constants::HAS_BALL_DISTANCE() { return (SETTINGS.getRobotHubMode() == Settings::BASESTATION) ? 0.11 : 0.12; }

// The max angle the ball can have to the robot for the robot to have the ball
double Constants::HAS_BALL_ANGLE() { return 0.10 * M_PI; }

std::map<int, bool> Constants::ROBOTS_WITH_WORKING_DRIBBLER() {
    static std::map<int, bool> workingDribblerRobots;
    workingDribblerRobots[0] = true;
    workingDribblerRobots[1] = true;
    workingDribblerRobots[2] = true;
    workingDribblerRobots[3] = true;
    workingDribblerRobots[4] = true;
    workingDribblerRobots[5] = true;
    workingDribblerRobots[6] = true;
    workingDribblerRobots[7] = true;
    workingDribblerRobots[8] = true;
    workingDribblerRobots[9] = true;
    workingDribblerRobots[10] = true;
    workingDribblerRobots[11] = true;
    workingDribblerRobots[12] = true;
    workingDribblerRobots[13] = true;
    workingDribblerRobots[14] = true;
    workingDribblerRobots[15] = true;

    return workingDribblerRobots;
}

// TODO: Make robot send this information instead of us hardcoding these values
std::map<int, bool> Constants::ROBOTS_WITH_WORKING_BALL_SENSOR() {
    static std::map<int, bool> workingBallSensorRobots;
    workingBallSensorRobots[0] = false;
    workingBallSensorRobots[1] = false;
    workingBallSensorRobots[2] = false;
    workingBallSensorRobots[3] = false;
    workingBallSensorRobots[4] = false;
    workingBallSensorRobots[5] = false;
    workingBallSensorRobots[6] = false;
    workingBallSensorRobots[7] = false;
    workingBallSensorRobots[8] = false;
    workingBallSensorRobots[9] = false;
    workingBallSensorRobots[10] = false;
    workingBallSensorRobots[11] = false;
    workingBallSensorRobots[12] = false;
    workingBallSensorRobots[13] = false;
    workingBallSensorRobots[14] = false;
    workingBallSensorRobots[15] = false;

    return workingBallSensorRobots;
}

// With the dribbler encoder, we can detect if the robot has the ball
std::map<int, bool> Constants::ROBOTS_WITH_WORKING_DRIBBLER_ENCODER() {
    static std::map<int, bool> workingDribblerEncoderRobots;
    workingDribblerEncoderRobots[0] = true;
    workingDribblerEncoderRobots[1] = true;
    workingDribblerEncoderRobots[2] = true;
    workingDribblerEncoderRobots[3] = true;
    workingDribblerEncoderRobots[4] = true;
    workingDribblerEncoderRobots[5] = true;
    workingDribblerEncoderRobots[6] = true;
    workingDribblerEncoderRobots[7] = true;
    workingDribblerEncoderRobots[8] = true;
    workingDribblerEncoderRobots[9] = true;
    workingDribblerEncoderRobots[10] = true;
    workingDribblerEncoderRobots[11] = true;
    workingDribblerEncoderRobots[12] = true;
    workingDribblerEncoderRobots[13] = true;
    workingDribblerEncoderRobots[14] = true;
    workingDribblerEncoderRobots[15] = true;

    return workingDribblerEncoderRobots;
}

std::map<int, bool> Constants::ROBOTS_WITH_KICKER() {
    static std::map<int, bool> kickerRobots;
    kickerRobots[0] = true;
    kickerRobots[1] = true;
    kickerRobots[2] = true;
    kickerRobots[3] = true;
    kickerRobots[4] = true;
    kickerRobots[5] = true;
    kickerRobots[6] = true;
    kickerRobots[7] = true;
    kickerRobots[8] = true;
    kickerRobots[9] = true;
    kickerRobots[10] = true;
    kickerRobots[11] = true;
    kickerRobots[12] = true;
    kickerRobots[13] = true;
    kickerRobots[14] = true;
    kickerRobots[15] = true;

    return kickerRobots;
}

std::map<int, int> Constants::ROBOTS_MAXIMUM_KICK_TIME() {
    static std::map<int, int> maximumKickTimes;
    maximumKickTimes[0] = 25;
    maximumKickTimes[1] = 31; // Tested: 6.1
    maximumKickTimes[2] = 25; // Tested
    maximumKickTimes[3] = 25;
    maximumKickTimes[4] = 25;
    maximumKickTimes[5] = 25; // Tested
    maximumKickTimes[6] = 25;
    maximumKickTimes[7] = 20; // Tested: 5.5
    maximumKickTimes[8] = 40; // Tested: 5 is actualy 60
    maximumKickTimes[9] = 25; // Tested: idk
    maximumKickTimes[10] = 25;
    maximumKickTimes[11] = 25;
    maximumKickTimes[12] = 25;
    maximumKickTimes[13] = 40; // Tested: 4.8 is actually 60
    maximumKickTimes[14] = 25;
    maximumKickTimes[15] = 25;

    return maximumKickTimes;
}


bool Constants::ROBOT_HAS_WORKING_BALL_SENSOR(int id) { return ROBOTS_WITH_WORKING_BALL_SENSOR()[id]; }

bool Constants::ROBOT_HAS_WORKING_DRIBBLER(int id) { return ROBOTS_WITH_WORKING_DRIBBLER()[id]; }

bool Constants::ROBOT_HAS_WORKING_DRIBBLER_ENCODER(int id) { return ROBOTS_WITH_WORKING_DRIBBLER_ENCODER()[id]; }

bool Constants::ROBOT_HAS_KICKER(int id) { return ROBOTS_WITH_KICKER()[id]; }

int Constants::ROBOT_MAXIMUM_KICK_TIME(int id) { return ROBOTS_MAXIMUM_KICK_TIME()[id]; }

QColor Constants::FIELD_COLOR() { return GRSIM() ? QColor(30, 30, 30, 255) : QColor(50, 0, 0, 255); }

QColor Constants::FIELD_LINE_COLOR() { return Qt::white; }

QColor Constants::ROBOT_COLOR_BLUE() { return {150, 150, 255, 255}; }

QColor Constants::ROBOT_COLOR_YELLOW() { return {255, 255, 0, 255}; }

QColor Constants::BALL_COLOR() { return {255, 120, 50, 255}; }

QColor Constants::TEXT_COLOR() { return Qt::white; }

QColor Constants::SELECTED_ROBOT_COLOR() { return Qt::magenta; }

std::vector<QColor> Constants::TACTIC_COLORS() { return {{255, 0, 255, 50}, {0, 255, 255, 50}, {255, 255, 0, 50}, {0, 255, 0, 50}, {0, 0, 255, 100}}; }

pidVals Constants::standardNumTreePID() { return GRSIM() ? pidVals(2.5, 0.0, 0) : pidVals(2.5, 0.0, 0); }

pidVals Constants::standardReceivePID() { return GRSIM() ? pidVals(4, 0, 0) : pidVals(4, 0, 0); }

pidVals Constants::standardInterceptPID() { return GRSIM() ? pidVals(6, 0, 1) : pidVals(6, 0, 1); }

pidVals Constants::standardKeeperPID() { return GRSIM() ? pidVals(2.5, 0.0, 0) : pidVals(2.5, 0.0, 0); }

pidVals Constants::standardKeeperInterceptPID() { return GRSIM() ? pidVals(6, 0, 1) : pidVals(6, 0, 1); }

std::vector<RuleSet> Constants::ruleSets() {
    return {{"default", 2, 6.5, 0.0, ROBOT_RADIUS(), true},
            {"halt", 0.0, 0.0, 0.0, -1, true},
            {"stop", 1.3, 0.0, 0.8, -1, false},
            {"ballplacement_them", 1.3, 6.5, 0.8, -1, true},
            {"ballplacement_us", 0.75 /*2.5*/, 6.5, 0.0, -1, true},
            {"kickoff", 1.5, 6.5, 0.5, 0.0, true}};
}

}  // namespace rtt::ai
