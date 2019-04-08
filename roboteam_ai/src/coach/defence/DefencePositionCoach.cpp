#include <utility>

//
// Created by rolf on 18-2-19.
//

#include "DefencePositionCoach.h"
#include "roboteam_ai/src/world/Field.h"
#include "roboteam_ai/src/control/ControlUtils.h"
#include "roboteam_ai/src/utilities/RobotDealer.h"
/// This is a class that computes useful lines and positions for computing defender positions
namespace rtt {
namespace ai {
namespace coach {
using util = control::ControlUtils;

DefencePositionCoach g_defensivePositionCoach;

bool DefencePositionCoach::DefenderBot::validPosition(const world::WorldData &world) {
    for (const auto &bot : world.us) {
        if ((bot.pos - targetPos).length() < 2*Constants::ROBOT_RADIUS()) {
            return false;
        }
    }
    return true;
}
world::Robot DefencePositionCoach::DefenderBot::toRobot() {
    world::Robot robot;
    robot.id = - 1;
    robot.pos = targetPos;
    robot.angle = orientation;
    return robot;
}
// pick position on the line depending on how aggressive we want to play. aggression factor 1 is very in your face, whilst 0 is as close as possible to the goal
Vector2 DefencePositionCoach::getPosOnLine(const Line &line, double aggressionFactor) {
    if (aggressionFactor < 0) {
        return line.second;
    }
    else if (aggressionFactor > 1) {
        return line.first;
    }
    return line.second + (line.first - line.second)*aggressionFactor;
}
// get the direction facing towards the end of the Line
double DefencePositionCoach::getOrientation(const Line &line) {
    return (line.second - line.first).angle();
}

//computes a line segment on which the entirety of openGoalSegment is blocked as seen from point with robots with radius collissionRadius
std::shared_ptr<Line> DefencePositionCoach::getBlockLineSegment(
        const Line &openGoalSegment, const Vector2 &point, double collisionRadius, double margin) {
    if (margin == - 1.0) { margin = collisionRadius; }

    Vector2 FurthestBlock = getBlockPoint(openGoalSegment, point, collisionRadius);
    Vector2 startPos =
            point + (FurthestBlock - point).stretchToLength(collisionRadius);//start should be out of collision radius
    // if the starting position is in the defence area you cannot 'squeeze a robot in between the position and the defence area
    if (world::field->pointIsInDefenceArea(startPos, true, margin)) {
        return nullptr;
    }
    //check intersections with defense area and shorten line if needed
    Line line = shortenLineForDefenseArea(startPos, FurthestBlock, margin);
    std::shared_ptr<Line> segment = std::make_shared<Line>(line);
    return segment;

}
/// computes the intersection of the bisector of the angle to OpenGoalsegment from point and the defence area line
std::shared_ptr<Vector2> DefencePositionCoach::blockOnDefenseLine(const Line &openGoalSegment,
        const Vector2 &point) {
    // margin by which we shift the defence area line forwards
    double margin = 0.15;
    double collisionRadius = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
    //compute the bisector
    Vector2 lineToSideOne = openGoalSegment.first - point;
    Vector2 lineToSideTwo = (openGoalSegment.second - point);
    Vector2 startPos = point + (lineToSideOne + lineToSideTwo).stretchToLength(collisionRadius);
    // if starting point is in the defence area there is no room for a robot to squeeze in
    if (world::field->pointIsInDefenceArea(startPos, true, margin)) {
        return nullptr;
    }
    Vector2 endPos = point + (lineToSideOne + lineToSideTwo)*0.5;// this defines the line on which the bisector lies.
    // now compute intersection with the defense area and return this (if it exists)
    return world::field->lineIntersectsWithDefenceArea(true, point, endPos, margin);
}
/// gets the furthest position at which an obstacle will block the entire Angle
/// intuitively you can understand this as the closest point to which a circle of collissionRadius 'fits' in between the two lines
Vector2 DefencePositionCoach::getBlockPoint(const Line &openGoalSegment, const Vector2 &point,
        double collisionRadius) {
    //compute the bisector of the angle of point and the two ends of the openGoalSegment
    Vector2 lineToSideOne = openGoalSegment.first - point;
    Vector2 lineToSideTwo = (openGoalSegment.second - point);
    Vector2 endPos = point + (lineToSideOne + lineToSideTwo)
            *0.5;// ending point on the bisector, which always just intersects the goalLine
    // compute the furthest distance at which the entire segment is blocked
    double theta = lineToSideOne.angle() - (endPos - point).angle(); // half of the angle of the bisector
    double collisionDist = collisionRadius/sin(theta);
    Vector2 FurthestBlock = point + Vector2(collisionDist, 0).rotate((endPos - point).angle());
    return FurthestBlock;
}

// if the line hits the defence area shorten it, otherwise just return the original line
Line DefencePositionCoach::shortenLineForDefenseArea(const Vector2 &lineStart, const Vector2 &lineEnd,
        double defenseMargin) {
    Line line;
    std::shared_ptr<Vector2> intersectPos = world::field->lineIntersectsWithDefenceArea(true, lineStart, lineEnd,
            defenseMargin);
    if (! intersectPos) {
        // return the original line
        line = std::make_pair(lineStart, lineEnd);
    }
    else {
        // it intersects with the defence area, so we use the intersect position
        line = std::make_pair(lineStart, *intersectPos);
    }
    return line;
}

std::vector<DefencePositionCoach::DefenderBot> DefencePositionCoach::decidePositions(int amount) {
    std::vector<DefenderBot> defenders;
    if (amount < 0) {
        std::cerr << "can't assign less than 0 Defender locations!!";
        return defenders;
    }
    world::WorldData simulatedWorld = world::world->getWorld();
    simulatedWorld.us.clear();
    //first we handle the most dangerous position first. This needs to be blocked completely
    DefenderBot crucialDefender=createBlockBall(simulatedWorld);
    defenders.push_back(crucialDefender);
    // for the remainder we look at the possiblePasses and block the most dangerous bots
    std::vector<PossiblePass> passes = createPassesSortedByDanger(simulatedWorld);
    while (defenders.size() != amount && ! passes.empty()) {
        std::shared_ptr<DefenderBot> defender;
        defender = tryBlockToGoal(passes[0], 0.0, simulatedWorld);//first try a complete direct block.
        if (! defender) {
            defender = tryBlockOnLine(passes[0], simulatedWorld); // if that doesn't work try blocking on the d-line
        }
        if (! defender) {
            defender = tryBlockPass(passes[0], simulatedWorld); // if that doesn't work try to prevent the pass
        }
        // if we find a defender add it to our queue and our simulatedWorld and update the passes we found
        if (defender){
            defenders.push_back(*defender);
            simulatedWorld.us.push_back(defender->toRobot());
            passes=createPassesSortedByDanger(simulatedWorld);
        }
        else{
            //remove the attacker from the simulated world as we cannot cover it anyways
            simulatedWorld=removeBotFromWorld(simulatedWorld,passes[0].toBot->id,false);
        }
    }
    return defenders;
}
world::WorldData DefencePositionCoach::removeBotFromWorld(world::WorldData world,int id,bool isUs){
    std::vector<world::Robot> bots=world.them;
    if (isUs) {
        bots = world.us;
    }
    auto endIt=std::remove_if(bots.begin(),bots.end(),[id](const world::Robot &bot){
      return id==bot.id;
    });

    if (isUs){
        world.us.clear();
        for (auto p=bots.begin(); p!=endIt; ++p){
            world.us.push_back(*p);
        }
    }
    else{
        world.them.clear();
        for (auto p=bots.begin(); p!=endIt; ++p){
            world.them.push_back(*p);
        }
    }
    return world;
}
Vector2 DefencePositionCoach::getMostDangerousPos(const world::WorldData &world) {
    return world.ball.pos;
}
std::vector<std::pair<PossiblePass, double>> DefencePositionCoach::createPassesAndDanger(
        const world::WorldData &world) {
    std::vector<std::pair<PossiblePass, double>> passWithScore;
    // check the passes from the robot towards every other bot and calculate their danger
    for (auto theirBot : world.them) {
        //TODO: ignore robots that we already have covered here
        PossiblePass pass(&theirBot, world.ball.pos);
        double danger = pass.score(world); // check how dangerous the pass is in our simulated world
        std::pair<PossiblePass, double> passPair = std::make_pair(pass, danger);
        passWithScore.push_back(passPair);
    }
    return passWithScore;

}
std::vector<PossiblePass> DefencePositionCoach::sortPassesByDanger(
        std::vector<std::pair<PossiblePass, double>> &passWithScore) {
    //order passes from most dangerous to least
    std::sort(passWithScore.begin(), passWithScore.end(),
            [](std::pair<PossiblePass, double> &left, std::pair<PossiblePass, double> &right) {
              return left.second > right.second;
            });
    std::vector<PossiblePass> passes;
    for (const auto &passScore : passWithScore) {
        passes.push_back(passScore.first);
    }
    return passes;
}
std::vector<PossiblePass> DefencePositionCoach::createPassesSortedByDanger(const rtt::ai::world::WorldData &world) {
    auto passes = createPassesAndDanger(world);
    return sortPassesByDanger(passes);
}
std::shared_ptr<DefencePositionCoach::DefenderBot> DefencePositionCoach::tryBlockToGoal(const PossiblePass &pass,
        double aggressionFactor, const world::WorldData &simulatedWorld) {
    // get the blockLine segment from the ending position of the pass
    auto visibleParts = world::field->getVisiblePartsOfGoal(true, pass.endPos, simulatedWorld);
    std::sort(visibleParts.begin(), visibleParts.end(),
            [](const Line &a, const Line &b) {
              return abs(a.second.y - a.first.y) > abs(b.second.y - b.first.y);
            });
    if (! visibleParts.empty()) {
        auto blockLine = getBlockLineSegment(visibleParts[0], pass.endPos);
        if (blockLine) {
            DefenderBot bot;
            bot.type = botType::BLOCKTOGOAL;
            bot.blockFromID = pass.toBot->id;
            bot.targetPos = getPosOnLine(*blockLine, aggressionFactor);
            bot.orientation = getOrientation(*blockLine);
            return std::make_shared<DefenderBot>(bot);
        }
        return nullptr;
    }
    return nullptr;
}

std::shared_ptr<DefencePositionCoach::DefenderBot> DefencePositionCoach::tryBlockOnLine(const PossiblePass &pass,
        const world::WorldData &simulatedWorld) {
    auto visibleParts = world::field->getVisiblePartsOfGoal(true, pass.endPos, simulatedWorld);
    std::sort(visibleParts.begin(), visibleParts.end(),
            [](const Line &a, const Line &b) {
              return abs(a.second.y - a.first.y) > abs(b.second.y - b.first.y);
            });
    if (! visibleParts.empty()) {
        auto blockPos = blockOnDefenseLine(visibleParts[0], pass.endPos);
        if (blockPos) {
            DefenderBot bot;
            bot.type = botType::BLOCKONLINE;
            bot.blockFromID = pass.toBot->id;
            bot.targetPos = *blockPos;
            //draw a line for easy orientation computation
            Line line = std::make_pair(pass.endPos, *blockPos);
            bot.orientation = getOrientation(line);
            return std::make_shared<DefenderBot>(bot);
        }
        return nullptr;
    }
    return nullptr;
}
std::shared_ptr<DefencePositionCoach::BlockPassBot> DefencePositionCoach::tryBlockPass(PossiblePass &pass,
        const world::WorldData &simulatedWorld) {
    if (pass.distance() < 2*Constants::ROBOT_RADIUS()) {
        BlockPassBot bot;
        bot.type = botType::BLOCKPASS;
        bot.blockFromID = pass.toBot->id;
        bot.targetPos = pass.posOnLine(0.5);
        bot.orientation = pass.faceLine();
        return std::make_shared<BlockPassBot>(bot);
    }
    return nullptr;
}
DefencePositionCoach::DefenderBot DefencePositionCoach::createBlockBall(const world::WorldData &simulatedWorld) {
    Vector2 mostDangerousPos=getMostDangerousPos(simulatedWorld);

    DefenderBot bot;
    bot.type=botType::BLOCKBALL;
    bot.targetPos=getBlockBallPos(mostDangerousPos);
    bot.blockFromID=-1;
    bot.blockFromID=world::world->whichRobotHasBall();
    bot.orientation;
    return bot;
}
}//coach
}//ai
}//rtt