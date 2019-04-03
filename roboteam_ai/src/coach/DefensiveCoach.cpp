//
// Created by rolf on 18-2-19.
//

#include "DefensiveCoach.h"
#include "roboteam_ai/src/utilities/Field.h"
#include "roboteam_ai/src/control/ControlUtils.h"
#include "roboteam_ai/src/utilities/RobotDealer.h"
#include "roboteam_ai/src/interface/drawer.h"
/// This is a class that returns the positions we want our defenders to be at for all defenders
namespace rtt {
namespace ai {
namespace coach {
using util = control::ControlUtils;

DefensiveCoach g_defensiveCoach;

DefensiveCoach::PossiblePass::PossiblePass(roboteam_msgs::WorldRobot _toBot, Vector2 ballPos)
        :
        toBot(_toBot),
        startPos(ballPos),
        endPos(g_defensiveCoach.computeSimpleReceivePos(ballPos, _toBot.pos)) { };

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
        std::pair<Vector2, Vector2> openGoalSegment, Vector2 point, double collisionRadius, double margin) {
    if (margin == - 1.0) { margin = collisionRadius; }

    Vector2 FurthestBlock = getBlockPoint(openGoalSegment, point, collisionRadius);
    Vector2 startPos =
            point + (FurthestBlock - point).stretchToLength(collisionRadius);//start should be out of collision radius
    // if the starting position is in the defence area you cannot 'squeeze a robot in between the position and the defence area
    if (Field::pointIsInDefenceArea(startPos, true, margin)) {
        return nullptr;
    }
    //check intersections with defense area and shorten line if needed
    std::pair<Vector2, Vector2> line = shortenLineForDefenseArea(startPos, FurthestBlock, margin);
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

    Vector2 FurthestBlock = getBlockPoint(openGoalSegment, point, collisionRadius);
    Vector2 startPos =
            point + (FurthestBlock - point).stretchToLength(collisionRadius);//start should be out of collision radius
    // if the starting position is in the defence area you cannot 'squeeze a robot in between the position and the defence area
    if (Field::pointIsInDefenceArea(startPos, true, margin)) {
        return blockPositions;
    }
    std::shared_ptr<Vector2> intersectPos = Field::lineIntersectsWithDefenceArea(true, startPos, FurthestBlock, margin);
    if (! intersectPos) {
        // else try double blocking ; intersect radius goes up to 2*robotRadius+ball radius
        Vector2 blockPos;
        collisionRadius = 2*Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
        FurthestBlock = getBlockPoint(openGoalSegment, point, collisionRadius);
        std::shared_ptr<Vector2> intersectDoublePos = Field::lineIntersectsWithDefenceArea(true, startPos,
                FurthestBlock,
                margin);
        // if the 2 block position intersects with the defence area put them around that point
        double orientations[] = {M_PI_2, - M_PI_2};
        double lineAngle = (FurthestBlock - startPos).angle();
        for (auto orientation : orientations) {
            if (intersectDoublePos) {
                blockPos = *intersectDoublePos + Vector2(Constants::ROBOT_RADIUS(), 0).rotate(lineAngle + orientation);
            }
            else {
                // just use the furthestblock we found
                blockPos = FurthestBlock + Vector2(Constants::ROBOT_RADIUS(), 0).rotate(lineAngle + orientation);
            }
            // checking if any of the positions we chose is too close to the defence area
            std::shared_ptr<Vector2> intersection = Field::lineIntersectsWithDefenceArea(true, point, blockPos, margin);
            // moving them back to the line around the defence area if they are too far
            if (intersection) { blockPos = *intersection; }
            blockPositions.push_back(blockPos);
        }
    }
    else {
        // we have only one intersection
        blockPositions.push_back(*intersectPos);
    }
    return blockPositions;

}

///scores a possible pass based on the pass and blocks that already have been decided
double DefensiveCoach::scorePossiblePass(PossiblePass pass, std::vector<Vector2> decidedBlocks) {
    double score = 1.0;
    score *= scoreForOpenGoalAngle(pass, decidedBlocks);
    score *= penaltyForBlocks(pass, decidedBlocks);
    score *= penaltyForPassDist(pass);
    return score;
}

std::vector<std::pair<Vector2, double>> DefensiveCoach::decideDefendersOnDefenseLine(int amount) {
    std::vector<std::pair<Vector2, double>> decidedBlocks;
    if (amount <= 0) {
        std::cerr << "Can't assign 0 or less Defender locations!!";
        return decidedBlocks;
    }
    //first we cover off the most dangerous position

    std::pair<Vector2, Vector2> goalSides = Field::getGoalSides(true);

    if (World::getBall() && Field::pointIsInField(World::getBall()->pos)) {
        Vector2 mostDangerousPos = World::getBall()->pos; //TODO: add pass detection/ ball possession here for the 'most dangerous position'
        // if the ball is reasonably on our half double block, otherwise block using only one robot
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
        std::vector<Vector2> blocks; // this data format is easier to work with
        for (auto decidedBlock: decidedBlocks) {
            blocks.push_back(decidedBlock.first);
        }
        // create possible passes ordered from most dangerous to least dangerous
        std::vector<std::pair<PossiblePass, double>> passWithScore = createPassesAndDanger(bots, blocks);
        std::vector<roboteam_msgs::WorldRobot> virtualBots = createVirtualBots(blocks);// we create virtual bots
        // check for all the passes if we can block the shot from the pass receive position
        for (const auto &bestPass: passWithScore) {
            // find largest visible Part using virtualBots
            std::vector<std::pair<Vector2, Vector2>> visibleParts = Field::getVisiblePartsOfGoal(true,
                    bestPass.first.endPos, virtualBots,
                    Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
            std::sort(visibleParts.begin(), visibleParts.end(),
                    [](const std::pair<Vector2, Vector2> &a, const std::pair<Vector2, Vector2> &b) {
                      return abs(a.second.y - a.first.y) > abs(b.second.y - b.first.y);
                    });
            // compute blockpoint for this point
            std::shared_ptr<Vector2> blockPoint = nullptr;
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
/// calculates the defender locations for all available defenders
void DefensiveCoach::updateDefenderLocations() {
    if (doUpdate) {
        doUpdate = false;
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
        visualizePoints();
        auto stop = std::chrono::high_resolution_clock::now();
        std::cout << "Computation time:" << (std::chrono::duration_cast<chrono::nanoseconds>(stop - start).count()/1000000.0) << std::endl;
    }
}

// functions that communicate with the skill
/// adds a defender to the available defendersList
void DefensiveCoach::addDefender(int id) {
    bool robotIsRegistered = std::find(defenders.begin(), defenders.end(), id) != defenders.end();
    if (! robotIsRegistered) {
        defenders.push_back(id);
        setDoUpdate();
        //std::cout << "registered defender id:" << id << std::endl;
    }
    else {
        std::cerr << "Defender is already registered, check your tree!!" << std::endl;
    }
}

/// removes a defender from the available id's
void DefensiveCoach::removeDefender(int id) {
    auto defender = std::find(defenders.begin(), defenders.end(), id);
    if (defender != defenders.end()) {
        defenders.erase(defender);
        setDoUpdate();
//        std::cout << "removed defender id:" << id << std::endl;
    }
    else {
        std::cerr << "Defender cannot be removed as it is not registered! Check your skill" << std::endl;
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

/// gets the furthest position at which an obstacle will block the entire Angle
Vector2 DefensiveCoach::getBlockPoint(std::pair<Vector2, Vector2> openGoalSegment, Vector2 point,
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

std::pair<Vector2, Vector2> DefensiveCoach::shortenLineForDefenseArea(Vector2 lineStart, Vector2 lineEnd,
        double defenseMargin) {
    std::pair<Vector2, Vector2> line;
    std::shared_ptr<Vector2> intersectPos = Field::lineIntersectsWithDefenceArea(true, lineStart, lineEnd,
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
/// scores a point based on the largest openGoal angle from that point
double DefensiveCoach::scoreForOpenGoalAngle(PossiblePass pass, std::vector<Vector2> decidedBlocks) {
    // create the 'virtual bots' that make up the blocks we have already decided on
    std::vector<roboteam_msgs::WorldRobot> virtualBots = createVirtualBots(decidedBlocks);
    auto keeper = World::getRobotForId(robotDealer::RobotDealer::getKeeperID(),
            true); //TODO: actually make sure keeper is running
    if (keeper) {
        virtualBots.push_back(*keeper);
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

    return largestOpenGoalAngle;
}
/// penalize points that cannot be passed to
double DefensiveCoach::penaltyForBlocks(PossiblePass pass, std::vector<Vector2> decidedBlocks) {
    // we half the score for every robot that blocks the pass
    // can be made to also take 'potential intercepts' into account better
    int amountOfBlocks = 0;
    double obstacleFactor = 0.5;
    for (auto block : decidedBlocks) {
        if (util::distanceToLineWithEnds(block, pass.startPos, pass.endPos)
                <= (Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS())) {
            amountOfBlocks ++;
        }
    }
    return pow(obstacleFactor, amountOfBlocks);
}
/// penalize points that are far away to pass
double DefensiveCoach::penaltyForPassDist(PossiblePass pass) {
    // between goodUntilPassDist and impossiblePassDist we define a linear line that we multiply by the value of the distance with it.
    // e.g. at the halfway point between the two points the penalty is factor 0.5
    double goodUntilPassDist = 6.0;
    double impossiblePassDist = 15.0;
    if (pass.distance() > goodUntilPassDist) {
        return (1 - (pass.distance() - goodUntilPassDist)/(impossiblePassDist - goodUntilPassDist));
    }
    return 1.0;
}
void DefensiveCoach::visualizePoints() {
    int i = 0;
    std::vector<std::pair<Vector2, QColor>> vis2;
    for (auto location : defenderLocations) {
        std::pair<Vector2, QColor> pair;
        int colourcount = 6;
        if (i%colourcount == 0) {
            pair = make_pair(location.second.first, Qt::green);
        }
        else if (i%colourcount == 1) {
            pair = make_pair(location.second.first, Qt::red);
        }
        else if (i%colourcount == 2) {
            pair = make_pair(location.second.first, Qt::blue);
        }
        else if (i%colourcount == 3) {
            pair = make_pair(location.second.first, Qt::darkYellow);
        }
        else if (i%colourcount == 4) {
            pair = make_pair(location.second.first, Qt::darkMagenta);
        }
        else if (i%colourcount == 5) {
            pair = make_pair(location.second.first, Qt::cyan);
        }
        vis2.emplace_back(pair);
        i ++;
    }
    ai::interface::Drawer::setTestPoints(vis2);
}
std::vector<roboteam_msgs::WorldRobot> DefensiveCoach::createVirtualBots(std::vector<Vector2> decidedBlocks) {
    std::vector<roboteam_msgs::WorldRobot> virtualBots;
    for (auto blockPos : decidedBlocks) {
        roboteam_msgs::WorldRobot bot;
        bot.id = - 1;
        bot.pos = blockPos;
        virtualBots.push_back(bot);
    }
    return virtualBots;
}
std::vector<std::pair<DefensiveCoach::PossiblePass, double>> DefensiveCoach::createPassesAndDanger(
        std::vector<roboteam_msgs::WorldRobot> bots, std::vector<Vector2> decidedBlocks) {
    std::vector<std::pair<PossiblePass, double>> passWithScore;
    // check the passes from the robot towards every other bot and calculate their danger
    for (auto bot : bots) {
        PossiblePass pass(bot, World::getBall()->pos);
        std::vector<Vector2> onlyPositions;
        for (auto block :decidedBlocks) {
            onlyPositions.push_back(block);
        }
        double danger = scorePossiblePass(pass, onlyPositions);
        std::pair<PossiblePass, double> passPair = std::make_pair(pass, danger);
        passWithScore.push_back(passPair);
    }
    //order passes from most dangerous to least
    std::sort(passWithScore.begin(), passWithScore.end(),
            [](std::pair<PossiblePass, double> &left, std::pair<PossiblePass, double> &right) {
              return left.second > right.second;
            });
    return passWithScore;
}
Vector2 DefensiveCoach::computeSimpleReceivePos(Vector2 startPos, Vector2 robotPos) {
    Vector2 receivePos =
            robotPos + (startPos - robotPos).stretchToLength(Constants::CENTRE_TO_FRONT() + Constants::BALL_RADIUS());
    return receivePos;
}
void DefensiveCoach::setDoUpdate() {
    doUpdate = true;
}
}//coach
}//ai
}//rtt