#include <roboteam_ai/src/control/ControlUtils.h>
#include "World.h"

namespace rtt {
namespace ai {

// define the static variables
roboteam_msgs::World World::world;
bool World::didReceiveFirstWorld = false;
std::map<int, double> World::OurBotsBall;
std::map<int, double> World::TheirBotsBall;

std::mutex World::worldMutex;

/// return the world message
const roboteam_msgs::World &World::get_world() {
    std::lock_guard<std::mutex> lock(worldMutex);
    return World::world;
}

/// set a world message
/// if there is an 'us' vector, it sets didReceiveWorld to true
void World::set_world(roboteam_msgs::World _world) {
    std::lock_guard<std::mutex> lock(worldMutex);

    if (! _world.us.empty()) {
        didReceiveFirstWorld = true;
    }
    if (!_world.ball.visible){
        _world.ball=updateBallPosition(_world);
    }
    updateBallPossession(_world);
    world = _world;
    printRobotsWithBall();
}

/// Returns Robot for an ID and team (if it exists) 
std::shared_ptr<roboteam_msgs::WorldRobot> World::getRobotForId(unsigned int id, bool robotIsOurTeam) {
    roboteam_msgs::World _world;
    {
        std::lock_guard<std::mutex> lock(worldMutex);
        _world = world;
    }
    const std::vector<roboteam_msgs::WorldRobot> &robots = robotIsOurTeam ? _world.us : _world.them;
    for (const auto &bot : robots) {
        if (bot.id == id) {
            return std::make_shared<roboteam_msgs::WorldRobot>(bot);
        }
    }
    return nullptr;
}

/// returns robots for multiple ids and a team (if they are exist, otherwise the non-existent robots are nullptr)
std::vector<roboteam_msgs::WorldRobot> World::getRobotsForId(std::set<unsigned int> ids, bool robotsAreOurTeam) {
    std::vector<roboteam_msgs::WorldRobot> robots;
    for (const unsigned int &id : ids) {
        auto robot = getRobotForId(id, robotsAreOurTeam);
        if (robot) {
            robots.push_back(*robot);
        }
    }
    return robots;
}

/// returns the robot from a given vector closest to a given point
std::shared_ptr<roboteam_msgs::WorldRobot> World::getRobotClosestToPoint(std::vector<roboteam_msgs::WorldRobot> robots,
        const Vector2 &point) {

    std::shared_ptr<roboteam_msgs::WorldRobot> closest_robot;
    double closest_robot_ds = std::numeric_limits<double>::max();

    for (roboteam_msgs::WorldRobot worldRobot : robots) {
        Vector2 pos(worldRobot.pos);
        if ((pos - point).length() < closest_robot_ds) {
            closest_robot = std::make_shared<roboteam_msgs::WorldRobot>(worldRobot);
            closest_robot_ds = (pos - point).length();
        }
    }
    return closest_robot;
}

/// returns the ball msg
std::shared_ptr<roboteam_msgs::WorldBall> World::getBall() {
    std::lock_guard<std::mutex> lock(worldMutex);
    return std::make_shared<roboteam_msgs::WorldBall>(world.ball);
}


double World::findBallDist(roboteam_msgs::WorldRobot &bot, roboteam_msgs::WorldBall &ball) {
    Vector2 robotPos = Vector2(bot.pos);
    double robotOrientation = bot.angle;
    Vector2 ballPos = ball.pos;
    Vector2 dribbleLeft = robotPos
            + Vector2(Constants::ROBOT_RADIUS(), 0).rotate(robotOrientation - Constants::DRIBBLER_ANGLE_OFFSET());
    Vector2 dribbleRight = robotPos
            + Vector2(Constants::ROBOT_RADIUS(), 0).rotate(robotOrientation + Constants::DRIBBLER_ANGLE_OFFSET());
    // if the ball is very close (or on top of the robot?)
    if (control::ControlUtils::pointInTriangle(ballPos, robotPos, dribbleLeft, dribbleRight)) {
        return 0.0;
    }
    else if (control::ControlUtils::pointInRectangle(ballPos, dribbleLeft, dribbleRight,
            dribbleRight + Vector2(Constants::MAX_BALL_BOUNCE_RANGE(), 0).rotate(robotOrientation),
            dribbleLeft + Vector2(Constants::MAX_BALL_BOUNCE_RANGE(), 0).rotate(robotOrientation))) {
        return control::ControlUtils::distanceToLine(ballPos, dribbleLeft, dribbleRight);
    }
    // default return
    return - 1.0;

}
void World::updateBallPossession(roboteam_msgs::World &_world) {
    OurBotsBall.clear();
    TheirBotsBall.clear();
    // calculate if a bot is in dribbling range and if so what range it is at.
    for (auto ourBot :_world.us) {
        double dist = findBallDist(ourBot, _world.ball);
        if (dist != - 1.0) {
            OurBotsBall[ourBot.id] = dist;
        }
    }
    for (auto theirBot: _world.them) {
        double dist = findBallDist(theirBot, _world.ball);
        if (dist != - 1.0) {
            TheirBotsBall[theirBot.id] = dist;
        }
    }
}

// uses MAX_BALL_RANGE as default max Dist
bool World::ourBotHasBall(int id, double maxDistToBall){
    std::lock_guard<std::mutex> lock(worldMutex);
    if (OurBotsBall.find(id)!=OurBotsBall.end()){
        if (OurBotsBall[id]<=maxDistToBall){
            return true;
        }
    }
    return false;
}
bool World::theirBotHasBall(int id, double maxDistToBall){
    std::lock_guard<std::mutex> lock(worldMutex);
    if (TheirBotsBall.find(id)!=TheirBotsBall.end()){
        if (TheirBotsBall[id]<=maxDistToBall){
            return true;
        }
    }
    return false;
}
// picks the bot that has the ball and is closest to it.
int World::whichBotHasBall(bool ourTeam) {
    std::lock_guard<std::mutex> lock(worldMutex);
    double maxDist=100;
    int bestId=-1;
    if (ourTeam) {
        for (auto bot: OurBotsBall) {
            if (bot.second < maxDist) {
                maxDist = bot.second;
                bestId = bot.first;
            }
        }
    }
    else{
        for (auto bot: TheirBotsBall) {
            if (bot.second < maxDist) {
                maxDist = bot.second;
                bestId = bot.first;
            }
        }
    }
    return bestId;

}
roboteam_msgs::WorldBall World::updateBallPosition(roboteam_msgs::World _world) {
    roboteam_msgs::WorldBall newBall=_world.ball;
    if (_world.ball.visible){
        return newBall;
    }
    // we set the ball velocity to 0
    newBall.vel.x=0;
    newBall.vel.y=0;
    // if the ball was dribbled in the previous world_state we set its position to be in front of the robot that was dribbling it
    if (!OurBotsBall.empty()||!TheirBotsBall.empty()){ // check if it was dribbled in last world state
        double maxDist=100;
        int bestId=-1;
        bool ourTeam=true;
        for (auto bot: OurBotsBall ) {
            if (bot.second < maxDist) {
                maxDist = bot.second;
                bestId = bot.first;
                ourTeam = true;
            }
        }
        for (auto bot: TheirBotsBall ) {
            if (bot.second < maxDist) {
                maxDist = bot.second;
                bestId = bot.first;
                ourTeam = false;
            }
        }
        if (bestId==-1){
            ROS_ERROR("WTF BALL HANDLING");
            return newBall;
        }
        // I can't use getRobotForId because of deadlocking and because i need to use the world in the packet
        roboteam_msgs::WorldRobot robot;
        const std::vector<roboteam_msgs::WorldRobot> &robots = ourTeam ? _world.us: _world.them;
        for (const auto &bot : robots) {
            if (bot.id == bestId) {
                robot=bot;
                break;
            }
        }
        // put the ball in front of the centre robot that is dribbling it.
        Vector2 ballPos=Vector2(robot.pos)+Vector2(Constants::CENTRE_TO_FRONT()+Constants::BALL_RADIUS(),0).rotate(robot.angle);
        newBall.pos=ballPos;
    }
    // else (not visible but not dribbled), we put it at its previous position
    else{
        newBall.pos = world.ball.pos;
    }
    return newBall;
}
/// returns all the robots in the field, both us and them.
std::vector<roboteam_msgs::WorldRobot> World::getAllRobots() {
    std::lock_guard<std::mutex> lock(worldMutex);

    std::vector<roboteam_msgs::WorldRobot> allRobots;
    allRobots.insert(allRobots.end(), world.us.begin(), world.us.end());
    allRobots.insert(allRobots.end(), world.them.begin(), world.them.end());
    return allRobots;
}
void World::printRobotsWithBall(){
    std::cout<<"BallPos: "<<world.ball.pos<<std::endl;
    std::cout<<"Us:"<<std::endl;
    for (auto bot : OurBotsBall){
        std::cout<<"ID: "<<bot.first<<" dist: "<<bot.second<<std::endl;
    }
    std::cout<<"Them:"<<std::endl;
    for (auto bot : TheirBotsBall){
        std::cout<<"ID: "<<bot.first<<" dist: "<<bot.second<<std::endl;
    }
}
} // ai
} // rtt