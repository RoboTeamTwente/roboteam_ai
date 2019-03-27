//
// Created by rolf on 18-2-19.
//

#include "DefensiveCoach.h"
#include "Field.h"
#include "../control/ControlUtils.h"
#include "RobotDealer.h"
#include "../interface/drawer.h"
/// This is a class that returns the positions we want our defenders to be at for all defenders
namespace rtt {
namespace ai {
namespace coach {
using util = control::ControlUtils;
// initializing static storage
std::vector<int> DefensiveCoach::defenders;
std::map<int, std::pair<Vector2, double>> DefensiveCoach::defenderLocations;

DefensiveCoach::PossiblePass::PossiblePass(roboteam_msgs::WorldRobot _toBot, Vector2 ballPos)
        :
        toBot(_toBot),
        startPos(ballPos),
        endPos(util::computeSimpleReceivePos(ballPos, _toBot.pos)) { };

double DefensiveCoach::PossiblePass::distance() {
    return (endPos - startPos).length();
}
bool DefensiveCoach::PossiblePass::obstacleObstructsPath(Vector2 obstPos, double obstRadius) {
    return util::distanceToLineWithEnds(obstPos, startPos, endPos) <= obstRadius;
}
int DefensiveCoach::PossiblePass::amountOfBlockers() {
    int total = 0;
    for (auto bot : World::get_world().us) {
        if (obstacleObstructsPath(bot.pos)) {
            total ++;
        }
    }
    return total;
}
// pick position on the line depending on how aggressive we want to play. aggression factor 1 is very in your face, whilst 0 is as close as possible to the goal

Vector2 DefensiveCoach::getPos(std::pair<Vector2, Vector2> line, double aggressionFactor) {
    if (aggressionFactor < 0) {
        return line.second;
    }
    else if (aggressionFactor > 1) {
        return line.first;
    }
    return line.second + (line.first - line.second)*aggressionFactor;
}
//computes a line segment on which the entirety of openGoalSegment is blocked as seen from point with robots with radius collissionRadius

std::shared_ptr<std::pair<Vector2, Vector2>> DefensiveCoach::getBlockLineSegment(
        std::pair<Vector2, Vector2> openGoalSegment, Vector2 point, double collisionRadius) {
    double margin = collisionRadius;
    //compute the bisector of the angle of point and the two ends of the openGoalSegment
    Vector2 lineToSideOne = openGoalSegment.first - point;
    Vector2 lineToSideTwo = (openGoalSegment.second - point);
    // the starting position of the line needs to be atleast one robotRadius towards the goal if we want to be able to block on the line
    Vector2 startPos =
            point + (lineToSideOne + lineToSideTwo).stretchToLength(collisionRadius); //start of the line/bisector
    // if the starting position is in the defence area you cannot 'squeeze a robot in between the position and the defence area
    if (Field::pointIsInDefenceArea(startPos, true, margin)) {
        return nullptr;
    }
    Vector2 endPos = point + (lineToSideOne + lineToSideTwo)
            *0.5;// ending point on the bisector, which always just intersects the goalLine
    // compute the furthest distance at which the entire segment is blocked
    double theta = lineToSideOne.angle() - (endPos - point).angle(); // half of the angle of the bisector
    double collisionDist = collisionRadius/sin(theta);
    Vector2 FurthestBlock = point + Vector2(collisionDist, 0).rotate((endPos - point).angle());
    //check intersections with defense area
    // we use margin is Robot radius so the robots are with no part of themselves inside of the defence area
    std::shared_ptr<Vector2> intersectPos = Field::lineIntersectsWithDefenceArea(true, startPos, FurthestBlock, margin);
    std::pair<Vector2, Vector2> line;
    if (! intersectPos) {
        // return the original line
        line = std::make_pair(startPos, FurthestBlock);
    }
    else {
        // it intersects with the defence area, so we use the intersect position
        line = std::make_pair(startPos, *intersectPos);
    }
    std::shared_ptr<std::pair<Vector2, Vector2>> segment = std::make_shared<std::pair<Vector2, Vector2>>(line);
    return segment;

}
/// computes the intersection of the bisector of the angle to OpenGoalsegment from point and the defence area line
std::shared_ptr<Vector2> DefensiveCoach::blockOnDefenseLine(std::pair<Vector2, Vector2> openGoalSegment,
        Vector2 point) {
    // margin by which we shift the defence area line forwards
    double margin = 0.15;
    double collisionRadius = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
    //compute the bisector
    Vector2 lineToSideOne = openGoalSegment.first - point;
    Vector2 lineToSideTwo = (openGoalSegment.second - point);
    Vector2 startPos = point + (lineToSideOne + lineToSideTwo).stretchToLength(collisionRadius);
    // if starting point is in the defence area there is no room for a robot to squeeze in
    if (Field::pointIsInDefenceArea(startPos, true, margin)) {
        return nullptr;
    }
    Vector2 endPos = point + (lineToSideOne + lineToSideTwo)*0.5;// this defines the line on which the bisector lies.
    // now compute intersection with the defense area and return this (if it exists)
    return Field::lineIntersectsWithDefenceArea(true, point, endPos, margin);
}
/// places two robots as a block such that they are closest to the goal but block
std::vector<Vector2> DefensiveCoach::doubleBlockOnDefenseLine(
        std::pair<Vector2, Vector2> openGoalSegment, Vector2 point) {
    std::vector<Vector2> blockPositions;
    double margin = 0.15;
    double collisionRadius = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
    //compute the bisector
    Vector2 lineToSideOne = openGoalSegment.first - point;
    Vector2 lineToSideTwo = (openGoalSegment.second - point).stretchToLength(lineToSideOne.length());
    Vector2 startPos = point + (lineToSideOne + lineToSideTwo).stretchToLength(collisionRadius);
    // can we 'squeeze' in a robot?
    if (Field::pointIsInDefenceArea(startPos, true, margin)) {
        return blockPositions;
    }
    Vector2 endPos = point + (lineToSideOne + lineToSideTwo)*0.5;// this defines the line on which the bisector lies.
    // now compute intersection with the defense area
    double theta = lineToSideOne.angle() - (endPos - startPos).angle();
    double collisionDist = collisionRadius/sin(theta);
    Vector2 FurthestBlock = point + Vector2(collisionDist, 0).rotate((endPos - startPos).angle());
    // if it intersects with defense area we are only going to use one robot to cover the entire goal
    std::shared_ptr<Vector2> intersectPos;
    intersectPos = Field::lineIntersectsWithDefenceArea(true, startPos, FurthestBlock, margin);
    if (! intersectPos) {
        // else try double blocking ; intersect radius goes up to 2*robotRadius+ball radius
        Vector2 posOne, posTwo;
        collisionDist = (2*Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS())/sin(theta);
        FurthestBlock = point + Vector2(collisionDist, 0).rotate((endPos - startPos).angle());
        std::shared_ptr<Vector2> centerPos = Field::lineIntersectsWithDefenceArea(true, startPos, FurthestBlock,
                margin);
        // if the 2 block position intersects with the defence area put them around that point
        if (centerPos) {
            posOne = *centerPos + Vector2(Constants::ROBOT_RADIUS(), 0).rotate((endPos - startPos).angle() + M_PI_2);
            posTwo = *centerPos + Vector2(Constants::ROBOT_RADIUS(), 0).rotate((endPos - startPos).angle() - M_PI_2);
        }
        else {
            // just use the furthestblock
            posOne = FurthestBlock + Vector2(Constants::ROBOT_RADIUS(), 0).rotate((endPos - startPos).angle() + M_PI_2);
            posTwo = FurthestBlock + Vector2(Constants::ROBOT_RADIUS(), 0).rotate((endPos - startPos).angle() - M_PI_2);
        }
        // checking if any of the positions we chose is too close to the defence area
        std::shared_ptr<Vector2> intersectPosOne = Field::lineIntersectsWithDefenceArea(true, point, posOne, margin);
        std::shared_ptr<Vector2> intersectPosTwo = Field::lineIntersectsWithDefenceArea(true, point, posTwo, margin);
        // moving them back to the line around the defence area if they are too far
        if (intersectPosOne) { posOne = *intersectPosOne; }
        if (intersectPosTwo) { posTwo = *intersectPosTwo; }
        blockPositions.push_back(posOne);
        blockPositions.push_back(posTwo);
    }
    else {
        // we have only one intersection
        blockPositions.push_back(*intersectPos);
    }
    return blockPositions;

}
std::shared_ptr<std::pair<Vector2, Vector2>> DefensiveCoach::blockBall() {
    if (World::getBall()) {
        auto goalSides = Field::getGoalSides(true);
        return getBlockLineSegment(goalSides, World::getBall()->pos,
                Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
    }
    return nullptr;
}

///returns all passes that are not obstructed
std::vector<DefensiveCoach::PossiblePass> DefensiveCoach::getPossiblePassesThem() {
    std::vector<PossiblePass> possiblePasses;
    //first find if one of their bots has the ball and if so which bot
    int theirBotWithBall = World::whichBotHasBall(false);
    //if they do not have the ball then we just return as it's no use checking the passes
    std::shared_ptr<roboteam_msgs::WorldRobot> passBot = World::getRobotForId(theirBotWithBall, false);
    if (theirBotWithBall == - 1 || ! passBot) {
        return possiblePasses;
    }
    // for every bot check if it blocks the line. We exclude the bot that passes since it for sure does not 'block' the ball
    // and can rotate around it.
    for (auto bot: World::get_world().them) {
        if (bot.id != theirBotWithBall) {
            PossiblePass pass(bot, World::getBall()->pos);
            int blockAmount = pass.amountOfBlockers();
            if (blockAmount == 0) {
                possiblePasses.push_back(pass);
            }
        }
    }
    return possiblePasses;
}
//scores a possible pass based on the pass and blocks that already have been decided
double DefensiveCoach::scorePossiblePass(PossiblePass pass, std::vector<Vector2> decidedBlocks) {
    double score = 1.0;

    // create the 'virtual bots' that make up the blocks we have already decided on
    std::vector<roboteam_msgs::WorldRobot> virtualBots;
    auto keeper = World::getRobotForId(robotDealer::RobotDealer::getKeeperID(),
            true); //TODO: check if the keeper is actually running on this
    if (keeper) {
        virtualBots.push_back(*keeper);
    }
    for (auto blockPos : decidedBlocks) {
        roboteam_msgs::WorldRobot bot;
        bot.id = - 1;
        bot.pos = blockPos;
        virtualBots.push_back(bot);
    }
    // get the visible parts and sort them from large to small from the end point of the pass (the point of shooting)
    std::vector<std::pair<Vector2, Vector2>> visibleParts = Field::getVisiblePartsOfGoal(true, pass.endPos, virtualBots,
            Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
    std::sort(visibleParts.begin(), visibleParts.end(),
            [](const std::pair<Vector2, Vector2> &a, const std::pair<Vector2, Vector2> &b) {
              return abs(a.second.y - a.first.y) > abs(b.second.y - b.first.y);
            });
    // set the largest open angle
    double largestOpenGoalAngle;
    if (visibleParts.empty()) {
        // if there is none we set it to a minimum (0.05 of the total goal angle)
        largestOpenGoalAngle = Field::getTotalGoalAngle(true, pass.endPos)*0.05;
    }
    else {
        double angleOne = (visibleParts[0].first - pass.endPos).angle();
        double angleTwo = (visibleParts[0].second - pass.endPos).angle();
        largestOpenGoalAngle = control::ControlUtils::angleDifference(control::ControlUtils::constrainAngle(angleOne),
                control::ControlUtils::constrainAngle(angleTwo));
    }

    score = score*largestOpenGoalAngle;

    // penalties. For every block we half the total score (so powers of two
    int amountOfBlocks = 0;
    double obstacleFactor = 0.5;
    for (auto block : decidedBlocks) {
        if (util::distanceToLineWithEnds(block, pass.startPos, pass.endPos)
                <= (Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS())) {
            amountOfBlocks ++;
        }
    }
    score = score*pow(obstacleFactor, amountOfBlocks);

    // between goodUntilPassDist and impossiblePassDist we define a linear line that we multiply by the value of the distance with it.
    // e.g. at the halfway point between the two points the penalty is factor 0.5
    double goodUntilPassDist = 6.0;
    double impossiblePassDist = 15.0;
    if (pass.distance() > goodUntilPassDist) {
        score = score*(1 - (pass.distance() - goodUntilPassDist)/(impossiblePassDist - goodUntilPassDist));
    }
    return score;
}

std::vector<std::pair<Vector2, double>> DefensiveCoach::decideDefendersOnDefenseLine(int amount) {
    std::vector<std::pair<Vector2, double>> decidedBlocks;
    if (amount <= 0) {
        std::cerr << "Can't assign 0 or less Defender locations!!";
        return decidedBlocks;
    }
    // we decide the lines on which we want the defenders to be
    //most importantly; make sure the ball can't be kicked into the goal directly; we put one or more robots as a direct block;

    std::pair<Vector2, Vector2> goalSides = Field::getGoalSides(true);

    if (World::getBall() && Field::pointIsInField(World::getBall()->pos)) {
        Vector2 mostDangerousPos = World::getBall()->pos; //TODO: add pass detection/ ball possession here for the 'most dangerous position'
        // if the ball is reasonably on our half double block, otherwise block using only one
        if (((goalSides.first - World::getBall()->pos).length() + (goalSides.second - World::getBall()->pos).length())
                < Field::get_field().field_length*0.85) {
            std::vector<Vector2> blockPositions = doubleBlockOnDefenseLine(goalSides, mostDangerousPos);
            for (auto blockposition : blockPositions) {
                decidedBlocks.push_back(std::make_pair(blockposition, (mostDangerousPos - blockposition).angle()));
            }
        }
        else {
            auto blockLineSegment = getBlockLineSegment(goalSides, mostDangerousPos,
                    Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
            if (blockLineSegment) {
                // most defensive block
                decidedBlocks.push_back(std::make_pair(blockLineSegment->second,
                        (blockLineSegment->second - blockLineSegment->first).angle()));
            }
        }
        // we cannot double block with only one robot:
        if (amount == 1 && decidedBlocks.size() != 1) {
            auto segment = getBlockLineSegment(goalSides, mostDangerousPos,
                    Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
            if (segment) {
                Vector2 pos = getPos(*segment, 0);
                return {std::make_pair(pos, (mostDangerousPos - pos).angle())};
            }
            return {};
        }

    }
    if (amount == decidedBlocks.size()) {
        return decidedBlocks;
    }
    // now we look for passes and cover off the good positions from there
    // we assign a position to one robot and then recompute the passes again
    // compute which bot has the ball. We remove this one because it cannot pass to itself
    int theirBallBotId = World::whichBotHasBall(false);
    auto bots = World::get_world().them;
    if (theirBallBotId != - 1) {
        auto botPos = std::find_if(bots.begin(), bots.end(),
                [theirBallBotId](roboteam_msgs::WorldRobot bot) { return bot.id == theirBallBotId; });
        if (botPos != bots.end()) {
            bots.erase(botPos);
        }
    }
    //TODO: add more structured collision avoidance (so no posses within 2 Robot Radii of eachother) to the algorithm
    // we loop until we have as many blocks as we need or until there are no more bots left to cover off
    while (decidedBlocks.size() != amount && ! bots.empty()) {
        std::vector<std::pair<PossiblePass, double>> passWithScore;
        // check the passes from the robot towards every other bot and calculate their danger
        for (auto bot : bots) {
            PossiblePass pass(bot, World::getBall()->pos);
            std::vector<Vector2> onlyPositions;
            for (auto block :decidedBlocks) {
                onlyPositions.push_back(block.first);
            }
            double danger = scorePossiblePass(pass, onlyPositions);
            std::pair<PossiblePass, double> passPair = std::make_pair(pass, danger);
            passWithScore.push_back(passPair);
        }
        //order passes from most dangerous to least dangerous
        std::sort(passWithScore.begin(), passWithScore.end(),
                [](std::pair<PossiblePass, double> &left, std::pair<PossiblePass, double> &right) {
                  return left.second > right.second;
                });

        // we create virtual bots
        std::vector<roboteam_msgs::WorldRobot> virtualBots;
        for (auto blockPos : decidedBlocks) {
            roboteam_msgs::WorldRobot bot;
            bot.id = - 1;
            bot.pos = blockPos.first;
            bot.w = blockPos.second;
            virtualBots.push_back(bot);
        }
        // for all the passes order them and check if we can block the shot from the pass receive position
        for (const auto &bestPass: passWithScore) {
            std::vector<std::pair<Vector2, Vector2>> visibleParts = Field::getVisiblePartsOfGoal(true,
                    bestPass.first.endPos, virtualBots,
                    Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
            std::sort(visibleParts.begin(), visibleParts.end(),
                    [](const std::pair<Vector2, Vector2> &a, const std::pair<Vector2, Vector2> &b) {
                      return abs(a.second.y - a.first.y) > abs(b.second.y - b.first.y);
                    });
            std::shared_ptr<Vector2> blockPoint = nullptr;
            // we block on the defense line
            if (! visibleParts.empty()) {
                blockPoint = blockOnDefenseLine(visibleParts[0], bestPass.first.endPos);
            }
            // we erase the robot from the positions to be searched as it is either impossible to block or we have blocked it
            auto botPos = std::find_if(bots.begin(), bots.end(),
                    [bestPass](roboteam_msgs::WorldRobot b) { return b.id == bestPass.first.toBot.id; });
            if (botPos != bots.end()) {
                bots.erase(botPos);
            }
            // if we find a pass which we can block we break the loop and recompute using this newfound pass
            if (blockPoint) {
                double orientation = (bestPass.first.endPos - *blockPoint).angle();
                decidedBlocks.push_back(std::make_pair(*blockPoint, orientation));
                if (decidedBlocks.size() == amount) {
                    return decidedBlocks;
                }
                break;
            }

        }
    }
    // damn, we still haven't found enough defensive positions
    if (decidedBlocks.size() < amount) {
        //TODO: add some logic to generate additional positions e.g. zone defence, or just covering off a robot twice
    }
    return decidedBlocks;
}

void DefensiveCoach::updateDefenderLocations() {
    auto start = std::chrono::high_resolution_clock::now();
    // clear the defenderLocations
    defenderLocations.clear();
    std::vector<int> availableDefenders = defenders;
    // decide the locations to defend
    std::vector<std::pair<Vector2, double>> positions = decideDefendersOnDefenseLine(availableDefenders.size());

    // the following algorithm takes the closest robot for each available defender to decide which robot goes where.
    // Since the points are ordered on priority from the above algorithm the most important points come first
    // It might be better to use an algorithm that is more complicated (e.g. hungarian) but then we might need some kind of system which gives the first points more 'priority'
    for (auto position : positions) {
        int bestId = - 1;
        double bestDist = 10000000000;
        for (int botId : availableDefenders) {
            auto bot = World::getRobotForId(botId, true);
            if (bot) {
                if ((position.first - bot->pos).length() < bestDist) {
                    bestId = botId;
                    bestDist = (position.first - bot->pos).length();
                }
            }
            else {
                std::cerr << "Could not find robot " << botId << " to defend!";
            }
        }
        if (bestId != - 1) {
            defenderLocations[bestId] = position;
            availableDefenders.erase(std::find(availableDefenders.begin(), availableDefenders.end(), bestId));
        }
        else {
            std::cerr << "Could not find a robot to defend location!!!";
            return;
        }
    }
    //visualization
    int i = 0;
    std::vector<std::pair<Vector2, QColor>> vis2;
    for (auto location : positions) {
        std::pair<Vector2, QColor> pair;
        int colourcount = 6;
        if (i%colourcount == 0) {
            pair = make_pair(location.first, Qt::green);
        }
        else if (i%colourcount == 1) {
            pair = make_pair(location.first, Qt::red);
        }
        else if (i%colourcount == 2) {
            pair = make_pair(location.first, Qt::blue);
        }
        else if (i%colourcount == 3) {
            pair = make_pair(location.first, Qt::darkYellow);
        }
        else if (i%colourcount == 4) {
            pair = make_pair(location.first, Qt::darkMagenta);
        }
        else if (i%colourcount == 5) {
            pair = make_pair(location.first, Qt::cyan);
        }
        vis2.emplace_back(pair);
        i ++;
    }
    ai::interface::Drawer::setTestPoints(vis2);
    auto stop = std::chrono::high_resolution_clock::now();
    std::cout << "Computation time:"
              << (std::chrono::duration_cast<chrono::nanoseconds>(stop - start).count()/1000000.0) << std::endl;
}

// functions that communicate with the skill
/// adds a defender to the available defendersList
void DefensiveCoach::addDefender(int id) {
    bool robotIsRegistered = std::find(defenders.begin(), defenders.end(), id) != defenders.end();
    if (! robotIsRegistered) {
        defenders.push_back(id);
//        std::cout << "registered defender id:" << id << std::endl;
    }
}

/// removes a defender from the available id's
void DefensiveCoach::removeDefender(int id) {
    auto defender = std::find(defenders.begin(), defenders.end(), id);
    if (defender != defenders.end()) {
        defenders.erase(defender);
//        std::cout << "removed defender id:" << id << std::endl;
    }
}

/// get the specific position of a defender with specified id
std::shared_ptr<std::pair<Vector2, double>> DefensiveCoach::getDefenderPosition(int id) {
    auto element = defenderLocations.find(id);
    if (element == defenderLocations.end()) {
        return nullptr;
    }
    else return std::make_shared<std::pair<Vector2, double>>(defenderLocations[id]);
}
}
}
}