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
    return (line.first - line.second).angle();
}

//computes a line segment on which the entirety of openGoalSegment is blocked as seen from point with robots with radius collissionRadius
std::shared_ptr<Line> DefencePositionCoach::getBlockLineSegment(
        const Line &openGoalSegment, const Vector2 &point, double collisionRadius, double margin) {
    if (margin == - 1.0) { margin = defenceLineMargin; }

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
    double margin = defenceLineMargin;
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
    return world::field->lineIntersectionWithDefenceArea(true, point, endPos, margin);
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
    std::shared_ptr<Vector2> intersectPos = world::field->lineIntersectionWithDefenceArea(true, lineStart, lineEnd,
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
    defenders.clear(); // we are recomputing the positions again
    if (amount <= 0) { return defenders; } // we don't actually need to calculate now.
    // we start by creating a simulated world in which we place our robots according to the most needed positions 1 by 1
    simulatedWorld = setupSimulatedWorld();
    blockMostDangerousPos(); //first we handle the most dangerous position first
    // for the remainder we look at the possiblePasses and block the most dangerous bots
    std::vector<PossiblePass> passes = createPassesSortedByDanger(simulatedWorld);
    while (static_cast<int>(defenders.size()) != amount && ! passes.empty()) {
        bool foundNewDefender = blockPass(passes[0]); // we try to cover the most dangerous pass in multiple ways
        // if we find a defender we need to recalculate the danger of our passes to reflect the new robot.
        if (!foundNewDefender) {
            // if we cannot find a way to cover it, we remove the attacker from the simulated world (otherwise we get 'stuck')
            simulatedWorld = removeBotFromWorld(simulatedWorld, passes[0].toBot.id, false);
            // this should pretty much never happen.
            std::cerr<<"Pass to robot"<< passes[0].toBot.id <<" removed in defensiveCoach!"<<std::endl;
        }
        passes = createPassesSortedByDanger(simulatedWorld);

    }
    return defenders;
}
world::WorldData DefencePositionCoach::removeBotFromWorld(world::WorldData world, int id, bool ourTeam) {
    std::vector<world::Robot> robots = ourTeam ? world.us : world.them;
    auto endIt = std::remove_if(robots.begin(), robots.end(), [id](const world::Robot &robot) {
      return id == robot.id;
    });

    if (ourTeam) {
        world.us.clear();
        for (auto p = robots.begin(); p != endIt; ++ p) {
            world.us.push_back(*p);
        }
    }
    else {
        world.them.clear();
        for (auto p = robots.begin(); p != endIt; ++ p) {
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
    for (const auto &theirBot : world.them) {
        //TODO: perhaps ignore robots we have already covered here. The score should be gutted regardless.
        PossiblePass pass(theirBot, world.ball.pos);
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
DefencePositionCoach::DefenderBot DefencePositionCoach::createBlockToGoal(const PossiblePass &pass,
        double aggressionFactor, const Line &blockLine) {
    DefenderBot bot;
    bot.type = botType::BLOCKTOGOAL;
    bot.blockFromID = pass.toBot.id;
    bot.targetPos = getPosOnLine(blockLine, aggressionFactor);
    bot.orientation = getOrientation(blockLine);
    return bot;
}
DefencePositionCoach::DefenderBot DefencePositionCoach::createBlockToGoal(const PossiblePass &pass,
        const Vector2 &position, const Line &blockLine) {
    DefenderBot bot;
    bot.type = botType::BLOCKTOGOAL;
    bot.blockFromID = pass.toBot.id;
    bot.targetPos = position;
    bot.orientation = getOrientation(blockLine);
    return bot;
}
DefencePositionCoach::DefenderBot DefencePositionCoach::createBlockOnLine(const PossiblePass &pass,
        const Vector2 &blockPos) {
    DefenderBot bot;
    bot.type = botType::BLOCKONLINE;
    bot.blockFromID = pass.toBot.id;
    bot.targetPos = blockPos;
    //draw a line for easy orientation computation
    Line line = std::make_pair(pass.endPos, blockPos);
    bot.orientation = getOrientation(line);
    return bot;
}
DefencePositionCoach::DefenderBot DefencePositionCoach::createBlockPass(PossiblePass &pass,
        const Vector2 &blockPoint) {
    DefenderBot bot;
    bot.type = botType::BLOCKPASS;
    bot.blockFromID = pass.toBot.id;
    bot.targetPos = blockPoint;
    bot.orientation = pass.faceLine();
    return bot;
}
std::shared_ptr<Vector2> DefencePositionCoach::blockOnPassLine(PossiblePass &pass,
        const world::WorldData &world) {
    //TODO: make a nice algorithm that can check the entire line and divide it into segments? This works for now t.m.
    if (pass.distance() >= 2*Constants::ROBOT_RADIUS()) {
        return std::make_shared<Vector2>(pass.posOnLine(0.5));
    }
    return nullptr;
}
std::shared_ptr<Vector2> DefencePositionCoach::blockOnDefenseAreaLine(const PossiblePass &pass,
        const world::WorldData &world) {
    auto visibleParts = world::field->getVisiblePartsOfGoal(true, pass.endPos, world);
    // get the largest segment
    std::sort(visibleParts.begin(), visibleParts.end(),
            [](const Line &a, const Line &b) {
              return abs(a.second.y - a.first.y) > abs(b.second.y - b.first.y);
            });
    if (! visibleParts.empty()) {
        return blockOnDefenseLine(visibleParts[0], pass.endPos);
    }
    return nullptr;
}
/// checks for a given pass in a simulatedWorld if we can block it's receiver shot to goal and returns a line on which to stand if this is the case
std::shared_ptr<Line> DefencePositionCoach::blockToGoalLine(const PossiblePass &pass,
        const world::WorldData &world) {
    // get the blockLine segment from the ending position of the pass
    auto visibleParts = world::field->getVisiblePartsOfGoal(true, pass.endPos, world);
    // get the largest segment (sort by size)
    std::sort(visibleParts.begin(), visibleParts.end(),
            [](const Line &a, const Line &b) {
              return abs(a.second.y - a.first.y) > abs(b.second.y - b.first.y);
            });
    if (! visibleParts.empty()) {
        auto blockLine = getBlockLineSegment(visibleParts[0], pass.endPos);
        return blockLine;
    }
    return nullptr;
}
/// searches the most dangerous position and then gets the segment which blocks that (if it exists/is possible)
std::shared_ptr<Line> DefencePositionCoach::blockBallLine(const world::WorldData &world) {
    Vector2 mostDangerousPos = getMostDangerousPos(world);
    if (world::field->pointIsInField(mostDangerousPos)) {
        return getBlockLineSegment(world::field->getGoalSides(true), mostDangerousPos);
    }
    return nullptr;
}
DefencePositionCoach::DefenderBot DefencePositionCoach::createBlockBall(
        const Line &blockLine) {
    // TODO: handle special handling for the case where we can't block using this bot (e.g. try to make keeper actively block?)
    DefenderBot bot;
    bot.type = botType::BLOCKBALL;
    bot.targetPos = findPositionForBlockBall(blockLine);
    bot.blockFromID = world::world->whichRobotHasBall(world::THEIR_ROBOTS) ? (world::world->whichRobotHasBall(
            world::THEIR_ROBOTS)->id) : (- 1);
    bot.orientation = getOrientation(blockLine);
    return bot;
}
Vector2 DefencePositionCoach::findPositionForBlockBall(const Line &blockLine) {
    // if the blocking position is way away from our goal keep the robot on our side
    double maxForwardLineX = maxX();
    if (blockLine.second.x > maxForwardLineX) {
        double fieldWidth = world::field->get_field().field_width;
        Vector2 bottomLine(maxForwardLineX, - fieldWidth*0.5), topLine(maxForwardLineX, fieldWidth*0.5);
        Vector2 intersect = control::ControlUtils::twoLineIntersection(blockLine.first, blockLine.second, bottomLine,
                topLine);
        if (intersect.y > fieldWidth*0.5 || intersect.y < - fieldWidth*0.5) {
            std::cerr << "Please don't send robots outside of the field" << std::endl;
            return getPosOnLine(blockLine, 0.1);
        }
        return intersect;
    }
    return getPosOnLine(blockLine, 0.1);
}
double DefencePositionCoach::maxX() {
    return world::field->get_field().field_length/10.0*-1.0;
}
world::WorldData DefencePositionCoach::getTheirAttackers(const world::WorldData &world) {
    std::vector<world::Robot> theirAttackers;
    for (const world::Robot &robot :world.them) {
        // we remove any attackers that are outside of the field or in our defence area
        if (! world::field->pointIsInDefenceArea(robot.pos, true, 0.04)
                && world::field->pointIsInField(robot.pos, 0.1)) {
            theirAttackers.push_back(robot);
        }
    }
    world::WorldData newWorld = world;
    newWorld.them = theirAttackers;
    return newWorld;
}
bool DefencePositionCoach::validNewPosition(const Vector2 &position, const world::WorldData &world) {
    if (position.x > maxX()) { return false; }
    double collisionRadius = calculationCollisionRad;// a little smaller than 2 robot radii so that we can make solid walls still
    for (const auto &robot: world.us) {
        if ((robot.pos - position).length() < collisionRadius) {
            return false;
        }
    }
    return true;
}
std::shared_ptr<double> DefencePositionCoach::pickNewPosition(const Line &line,
        const world::WorldData &world) {
    //search a position on the line on which we can position.
    double lineLen = (line.second - line.first).length();
    double minSkipDist = 0.05;
    double skipDist = max(minSkipDist, lineLen/30.0);
    // try positions on the line by skipping forward skipDist every time. If one works return
    for (int aggresionFactor = 0; aggresionFactor < 1/skipDist; ++ aggresionFactor) {
        if (validNewPosition(getPosOnLine(line, aggresionFactor*skipDist), world)) {
            std::shared_ptr<double> point = std::make_shared<double>(aggresionFactor);
            return point;
        }
    }
    // the whole line is completely blocked (atleast on the points we tested)
    return nullptr;

}
std::shared_ptr<Vector2> DefencePositionCoach::pickNewPosition(PossiblePass pass,
        const world::WorldData &world) {
    std::shared_ptr<Vector2> point = nullptr;
    double segments = 30.0;
    // we pick new points on which we can defend preferably as close as possible to the middle of the pass.
    for (int j = 0; j <= segments*0.5; ++ j) {
        if (validNewPosition(pass.posOnLine(0.5 + j/segments), world)) {
            return std::make_shared<Vector2>(pass.posOnLine(0.5 + j/segments));
        }
        else if (validNewPosition(pass.posOnLine(0.5 - j/segments), world)) {
            return std::make_shared<Vector2>(pass.posOnLine(0.5 - j/segments));
        }
    }
    return nullptr;

}

world::WorldData DefencePositionCoach::setupSimulatedWorld() {
    world::WorldData sWorld = world::world->getWorld();
    sWorld.us.clear();
    sWorld = getTheirAttackers(sWorld);    // we select only the relevant robots
    return sWorld;
}
void DefencePositionCoach::blockMostDangerousPos() {
    //block the most dangerous position to the goal completely if possible.
    // check if it is possible to find a position that is legal
    auto crucialBlock = blockBallLine(simulatedWorld);
    if (crucialBlock) {
        DefenderBot crucialDefender = createBlockBall(*crucialBlock);// if so, create a defender
        addDefender(crucialDefender);
    }
}

bool DefencePositionCoach::blockPass(PossiblePass pass) {
    // this function tries multiple ways to block a pass. Returns true if it succeeded, false if it fails.
    // first try blocking the goal vision of the robot being passed to
    auto blockLine = blockToGoalLine(pass, simulatedWorld);
    if (blockLine) {
        // try to find a position on the line that is valid
        auto aggressionFactor = pickNewPosition(*blockLine, simulatedWorld);
        if (aggressionFactor) {
            DefenderBot defender = createBlockToGoal(pass, *aggressionFactor, *blockLine);
            addDefender(defender);
            return true;
        }
        // try putting it on the defence Line instead (as the robot is very likely far away
        double fieldWidth=world::field->get_field().field_width;
        Vector2 bottomLine(maxX(), - fieldWidth*0.5), topLine(maxX(), fieldWidth*0.5);
        Vector2 intersectPos=control::ControlUtils::twoLineIntersection(blockLine->first,blockLine->second,bottomLine,topLine);
        if(validNewPosition(intersectPos,simulatedWorld))
        {
            DefenderBot defender = createBlockToGoal(pass,intersectPos, *blockLine);
            addDefender(defender);
            return true;
        }
    }
    // then try blocking on the defense line (closer to the line) if that is possible
    auto blockPos = blockOnDefenseAreaLine(pass, simulatedWorld);
    if (blockPos) {
        if (validNewPosition(*blockPos, simulatedWorld)) {
            DefenderBot defender = createBlockOnLine(pass, *blockPos);
            addDefender(defender);
            return true;
        }
    }
    // then try to intercept the pass, we try to find a spot along the pass where we can stand
    auto passBlock = pickNewPosition(pass, simulatedWorld);
    if (passBlock) {
        DefenderBot defender = createBlockPass(pass, *passBlock);
        addDefender(defender);
        return true;
    }
    return false;
}
// if we add a defender we want to both add it to simulation and our stored defender array for return
void DefencePositionCoach::addDefender(DefenderBot defender) {
    simulatedWorld.us.push_back(defender.toRobot());
    defenders.push_back(defender);
}
}//coach
}//ai
}//rtt