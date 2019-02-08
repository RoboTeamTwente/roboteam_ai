//
// Created by baris on 6-12-18.
//

#include "Coach.h"
#include "../interface/InterfaceValues.h"
namespace rtt {
namespace ai {
namespace coach {

using dealer = robotDealer::RobotDealer;
std::map<int, int> Coach::defencePairs;

std::vector<int> Coach::defenders = {};
std::vector<int> Coach::robotsInFormation = {};


bool Coach::readyToReceivePass;
int Coach::robotBeingPassedTo;
bool Coach::passed;

int Coach::pickOffensivePassTarget(int selfID, std::string roleName) {

    // Get the other robots in that tactic
    std::string tacticName = dealer::getTacticNameForRole(std::move(roleName));
    auto tacticMates = dealer::findRobotsForTactic(tacticName);

    // Pick a free one TODO make better
    for (auto bot : tacticMates) {
        if (bot != selfID) {
            return bot;
//            if (control::ControlUtils::hasClearVision(selfID, bot, World::get_world(), 2)) {
//                return bot;
//            }
        }
    }
    return - 1;
}
int Coach::pickDefensivePassTarget(int selfID) {

    auto world = World::get_world();
    auto us = world.us;
    int safelyness = 3;
    while (safelyness > 0) {
        for (auto friendly : us) {
            if (control::ControlUtils::hasClearVision(selfID, friendly.id, world, safelyness)) {
                return friendly.id;
            }
        }
        safelyness --;
    }
    return - 1;
}
int Coach::pickHarassmentTarget(int selfID) {
    auto world = World::get_world();
    auto them = world.them;
    dangerfinder::DangerData dangerData = dangerfinder::DangerFinder::instance().getMostRecentData();
    std::vector<int> dangerList = dangerData.dangerList; // A list of robot IDs, sorted from most to least dangerous

    return *dangerList.begin();
}

int Coach::whichRobotHasBall(bool isOurTeam) {
    roboteam_msgs::World world = World::get_world();
    std::vector<roboteam_msgs::WorldRobot> robots;
    if (isOurTeam) {
        robots = world.us;
    }
    else {
        robots = world.them;
    }

    for (auto &robot:robots) {
        if (doesRobotHaveBall(robot.id, isOurTeam)) {
            return robot.id;
        }
    }
    return - 1;
}

int Coach::doesRobotHaveBall(unsigned int robotID, bool isOurTeam) {
    return doesRobotHaveBall(robotID, isOurTeam, rtt::ai::constants::MAX_BALL_RANGE, rtt::ai::constants::HAS_BALL_ANGLE);
}

int Coach::doesRobotHaveBall(unsigned int robotID, bool isOurTeam, double checkDist) {
    return doesRobotHaveBall(robotID, isOurTeam, checkDist, rtt::ai::constants::HAS_BALL_ANGLE);
}

int Coach::doesRobotHaveBall(unsigned int robotID, bool isOurTeam, double checkDist, double checkAngle) {
    auto robot = World::getRobotForId(robotID, isOurTeam);
    Vector2 ballPos;
    if (World::getBall())
        ballPos = World::getBall().get()->pos;
    else return 0;

    if (robot &&  World::getBall()) {
        Vector2 deltaPos = (ballPos-robot->pos);
        double dist = deltaPos.length();
        double angle = deltaPos.angle();
        double robotAngle = robot->angle;

        if (angle<0) {
            angle += 2*M_PI;
        }
        if (robotAngle<0) {
            robotAngle += 2*M_PI;
        }

        return ((dist<checkDist) && (fabs(angle-robotAngle)<checkAngle));
    }
    return false;
}

int Coach::pickOpponentToCover(int selfID) {
    dangerfinder::DangerData DangerData = dangerfinder::DangerFinder::instance().getMostRecentData();
    std::vector<int> dangerList = DangerData.dangerList;
    for (int &opponentID : dangerList) {
        if (defencePairs.find(opponentID) == defencePairs.end()) {
            if (! doesRobotHaveBall(static_cast<unsigned int>(opponentID), false)) {
                return opponentID;
            }
        }
        else if (defencePairs[opponentID] == selfID) {
            return opponentID;
        }
    }

    return - 1;
}

Vector2 Coach::getPositionBehindBallToGoal(double distanceBehindBall, bool ourGoal) {
    const Vector2 &goal = (ourGoal ? Field::get_our_goal_center : Field::get_their_goal_center)();
    return getPositionBehindBallToPosition(distanceBehindBall, goal);
}

Vector2 Coach::getPositionBehindBallToRobot(double distanceBehindBall, bool ourRobot, const unsigned int &robotID) {
    Vector2 robot;
    if (World::getRobotForId(robotID, ourRobot))
        robot = World::getRobotForId(robotID, ourRobot).get()->pos;
    else
        return Vector2();
    return getPositionBehindBallToPosition(distanceBehindBall, robot);
}

Vector2 Coach::getPositionBehindBallToPosition(double distanceBehindBall, const Vector2 &position) {
    const Vector2 &ball = static_cast<Vector2>(World::getBall()->pos);
    return ball + (ball - position).stretchToLength(distanceBehindBall);
}

bool Coach::isRobotBehindBallToGoal(double distanceBehindBall, bool ourGoal, const Vector2 &robotPosition) {
    const Vector2 &goal = (ourGoal ? Field::get_our_goal_center : Field::get_their_goal_center)();
    return isRobotBehindBallToPosition(distanceBehindBall, goal, robotPosition);
}

bool Coach::isRobotBehindBallToRobot(double distanceBehindBall, bool ourRobot, const unsigned int &robotID,
        const Vector2 &robotPosition) {
    Vector2 robot;
    if (World::getRobotForId(robotID, ourRobot))
        robot = World::getRobotForId(robotID, ourRobot).get()->pos;
    else
        return false;
    return isRobotBehindBallToPosition(distanceBehindBall, robot, robotPosition);
}

bool Coach::isRobotBehindBallToPosition(double distanceBehindBall, const Vector2 &position,
    const Vector2 &robotPosition) {
    const Vector2 &ball = static_cast<Vector2>(World::getBall()->pos);
    Vector2 behindBallPosition = getPositionBehindBallToPosition(distanceBehindBall, position);
    Vector2 deltaBall = behindBallPosition - ball;

    return (control::ControlUtils::pointInTriangle(robotPosition, ball, ball + (deltaBall).rotate(M_PI*0.17).scale(2.0),
            ball + (deltaBall).rotate(M_PI*- 0.17).scale(2.0)));
}

std::pair<int, bool> Coach::getRobotClosestToBall() {

    auto closestUs = World::getRobotClosestToPoint(World::get_world().us, World::getBall()->pos);
    auto closestThem = World::getRobotClosestToPoint(World::get_world().them, World::getBall()->pos);

    auto distanceToBallUs = (Vector2(closestUs->pos).dist(Vector2(World::getBall()->pos)));
    auto distanceToBallThem = (Vector2(closestThem->pos).dist(Vector2(World::getBall()->pos)));

    roboteam_msgs::WorldRobot closestRobot;
    bool weAreCloser;
    if (distanceToBallUs < distanceToBallThem) {
        closestRobot = * closestUs;
        weAreCloser = true;
    } else {
        closestRobot = * closestThem;
        weAreCloser = false;
    }

    return std::make_pair(closestRobot.id, weAreCloser);
}


Vector2 Coach::getDefensivePosition(int robotId) {
    addDefender(robotId);
    auto me = World::getRobotForId(robotId, true);
    auto field = Field::get_field();
    double targetLocationY = field.field_length/4 - (field.field_length/2);

    for (int i = 0; i<defenders.size(); i++) {
        if (defenders.at(i) == robotId) {
            return {targetLocationY, ((field.field_width/(defenders.size() + 1))*(i+1)) - field.field_width/2 };
        }
    }
    return {targetLocationY, 0};
}

void Coach::addDefender(int id) {
    bool robotIsRegistered = std::find(defenders.begin(), defenders.end(), id) != defenders.end();
    if (!robotIsRegistered) defenders.push_back(id);
}


void Coach::removeDefender(int id) {
    auto defender = std::find(defenders.begin(), defenders.end(), id);
    if (defender != defenders.end()) {
        defenders.erase(defender);
    }
}

Vector2 Coach::getRobotPositionClosestToPositionPosition(std::vector<roboteam_msgs::WorldRobot> &robots,
                                                         Vector2 position, bool includeSamePosition) {

    double distance = 999999;
    Vector2 pos = {999, 999};
    for (auto &bot : robots) {
        const Vector2 deltaPos = position - bot.pos;
        double dPLength = abs(deltaPos.length());
        if (dPLength < distance) {
            if (dPLength > 0.05 || includeSamePosition) {
                distance = dPLength;
                pos = bot.pos;
            }
        }
    }
    return pos;

}
void Coach::addFormationRobot(int id) {
    bool robotIsRegistered = std::find(robotsInFormation.begin(), robotsInFormation.end(), id) != robotsInFormation.end();
    if (!robotIsRegistered) robotsInFormation.push_back(id);
}

void Coach::removeFormationRobot(int id) {
    auto formationRobot = std::find(robotsInFormation.begin(), robotsInFormation.end(), id);
    if (formationRobot != robotsInFormation.end()) {
        robotsInFormation.erase(formationRobot);
    }
}

Vector2 Coach::getFormationPosition(int robotId) {
    addFormationRobot(robotId);
    auto me = World::getRobotForId(robotId, true);
    auto field = Field::get_field();
    double targetLocationY = field.field_length/4 - (field.field_length/2);
    for (int i = 0; i<robotsInFormation.size(); i++) {
        if (robotsInFormation.at(i) == robotId) {
            return {targetLocationY, ((field.field_width/(robotsInFormation.size() + 1))*(i+1)) - field.field_width/2 };
        }
    }
    return {targetLocationY, 0};
}

// --- Pass functions --- //

void Coach::resetPass() {
    setPassed(false);
    setReadyToReceivePass(false);
    setRobotBeingPassedTo(-1);
}

int Coach::initiatePass() {
    resetPass();

    // TODO: More logic to decide which robot to pass to. Possibly split initiate in initiate and findRobotToPassTo
    int robotBeingPassedTo = Field::getRobotClosestToGoal(true, false);
    setRobotBeingPassedTo(robotBeingPassedTo);
    return robotBeingPassedTo;
}

bool Coach::isReadyToReceivePass() {
    return readyToReceivePass;
}

void Coach::setReadyToReceivePass(bool readyToReceivePass) {
    Coach::readyToReceivePass = readyToReceivePass;
}

int Coach::getRobotBeingPassedTo() {
    return robotBeingPassedTo;
}

void Coach::setRobotBeingPassedTo(int robotBeingPassedTo) {
    Coach::robotBeingPassedTo = robotBeingPassedTo;
}

bool Coach::isPassed() {
    return passed;
}

void Coach::setPassed(bool passed) {
    Coach::passed = passed;
}

Vector2 Coach::getBallPlacementPos(){
    return interface::InterfaceValues::getBallPlacementTarget();
}

Vector2 Coach::getBallPlacementBeforePos(Vector2 ballPos){
    Vector2 PlacePos=interface::InterfaceValues::getBallPlacementTarget();
    Vector2 targetPos=ballPos + (PlacePos - ballPos).stretchToLength(constants::BP_MOVE_TOWARDS_DIST);
    return targetPos;
}
Vector2 Coach::getBallPlacementAfterPos(Vector2 ballPos, double RobotAngle){
    Vector2 targetPos=interface::InterfaceValues::getBallPlacementTarget() + Vector2(constants::BP_MOVE_BACK_DIST,0).rotate(RobotAngle+M_PI);
    return targetPos;
}
std::shared_ptr<roboteam_msgs::WorldRobot> Coach::getRobotClosestToBall(bool isOurTeam) {
    return World::getRobotClosestToPoint(isOurTeam ? World::get_world().us : World::get_world().them, World::getBall()->pos);
}
} //control
} //ai
} //rtt
