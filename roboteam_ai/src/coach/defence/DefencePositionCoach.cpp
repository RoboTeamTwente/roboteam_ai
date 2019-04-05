#include <utility>

//
// Created by rolf on 18-2-19.
//

#include "DefencePositionCoach.h"
#include "roboteam_ai/src/utilities/Field.h"
#include "roboteam_ai/src/control/ControlUtils.h"
#include "roboteam_ai/src/utilities/RobotDealer.h"
/// This is a class that computes useful lines and positions for computing defender positions
namespace rtt {
namespace ai {
namespace coach {
using util = control::ControlUtils;

DefencePositionCoach g_defensivePositionCoach;

DefencePositionCoach::PossiblePass::PossiblePass(roboteam_msgs::WorldRobot _toBot, const Vector2& ballPos)
        :
        toBot(_toBot),
        startPos(ballPos),
        endPos(g_defensivePositionCoach.computeSimpleReceivePos(ballPos, _toBot.pos)) { };

double DefencePositionCoach::PossiblePass::distance() {
    return (endPos - startPos).length();
}
bool DefencePositionCoach::PossiblePass::obstacleObstructsPath(const Vector2& obstPos, double obstRadius) {
    return util::distanceToLineWithEnds(obstPos, startPos, endPos) <= obstRadius;
}
int DefencePositionCoach::PossiblePass::amountOfBlockers() {
    int total = 0;
    for (auto bot : World::get_world().us) {
        if (obstacleObstructsPath(bot.pos)) {
            total ++;
        }
    }
    return total;
}
// pick position on the line depending on how aggressive we want to play. aggression factor 1 is very in your face, whilst 0 is as close as possible to the goal
Vector2 DefencePositionCoach::getPosOnLine(const Line& line, double aggressionFactor) {
    if (aggressionFactor < 0) {
        return line.second;
    }
    else if (aggressionFactor > 1) {
        return line.first;
    }
    return line.second + (line.first - line.second)*aggressionFactor;
}
// get the direction facing towards the end of the Line
double DefencePositionCoach::getOrientation(const Line& line) {
    return (line.second-line.first).angle();

}
Vector2 DefencePositionCoach::computeSimpleReceivePos(const Vector2& startPos, const Vector2& robotPos) {
    Vector2 receivePos =
            robotPos + (startPos - robotPos).stretchToLength(Constants::CENTRE_TO_FRONT() + Constants::BALL_RADIUS());
    return receivePos;
}

//computes a line segment on which the entirety of openGoalSegment is blocked as seen from point with robots with radius collissionRadius
std::shared_ptr<Line> DefencePositionCoach::getBlockLineSegment(
        const Line& openGoalSegment, const Vector2& point, double collisionRadius, double margin) {
    if (margin == - 1.0) { margin = collisionRadius; }

    Vector2 FurthestBlock = getBlockPoint(openGoalSegment, point, collisionRadius);
    Vector2 startPos =
            point + (FurthestBlock - point).stretchToLength(collisionRadius);//start should be out of collision radius
    // if the starting position is in the defence area you cannot 'squeeze a robot in between the position and the defence area
    if (Field::pointIsInDefenceArea(startPos, true, margin)) {
        return nullptr;
    }
    //check intersections with defense area and shorten line if needed
    Line line = shortenLineForDefenseArea(startPos, FurthestBlock, margin);
    std::shared_ptr<Line> segment = std::make_shared<Line>(line);
    return segment;

}
/// computes the intersection of the bisector of the angle to OpenGoalsegment from point and the defence area line
std::shared_ptr<Vector2> DefencePositionCoach::blockOnDefenseLine(const Line& openGoalSegment,
        const Vector2& point) {
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
std::vector<Vector2> DefencePositionCoach::doubleBlockOnDefenseLine(
        const Line& openGoalSegment, const Vector2& point) {
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
double DefencePositionCoach::scorePossiblePass(const PossiblePass& pass, const std::vector<Vector2>& decidedBlocks) {
    double score = 1.0;
    score *= scoreForOpenGoalAngle(pass, decidedBlocks);
    score *= penaltyForBlocks(pass, decidedBlocks);
    score *= penaltyForPassDist(pass);
    return score;
}

std::vector<std::pair<Vector2, double>> DefencePositionCoach::decideDefendersOnDefenseLine(int amount) {
    std::vector<std::pair<Vector2, double>> decidedBlocks;
    if (amount <= 0) {
        std::cerr << "Can't assign 0 or less Defender locations!!";
        return decidedBlocks;
    }
    //first we cover off the most dangerous position

    Line goalSides = Field::getGoalSides(true);

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
                        (blockLineSegment->first - blockLineSegment->second).angle()));
            }
        }
        // we cannot double block with only one robot:
        if (amount == 1 && decidedBlocks.size() != 1) {
            auto segment = getBlockLineSegment(goalSides, mostDangerousPos,
                    Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
            if (segment) {
                Vector2 pos = getPosOnLine(*segment, 0);
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
            std::vector<Line> visibleParts = Field::getVisiblePartsOfGoal(true,
                    bestPass.first.endPos, virtualBots,
                    Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
            std::sort(visibleParts.begin(), visibleParts.end(),
                    [](const Line &a, const Line &b) {
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

/// gets the furthest position at which an obstacle will block the entire Angle
Vector2 DefencePositionCoach::getBlockPoint(const Line& openGoalSegment, const Vector2& point,
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

Line DefencePositionCoach::shortenLineForDefenseArea(const Vector2& lineStart, const Vector2& lineEnd,
        double defenseMargin) {
    Line line;
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
double DefencePositionCoach::scoreForOpenGoalAngle(const PossiblePass& pass, const std::vector<Vector2>& decidedBlocks) {
    // create the 'virtual bots' that make up the blocks we have already decided on
    std::vector<roboteam_msgs::WorldRobot> virtualBots = createVirtualBots(decidedBlocks);
    auto keeper = World::getRobotForId(robotDealer::RobotDealer::getKeeperID(),
            true); //TODO: actually make sure keeper is running
    if (keeper) {
        virtualBots.push_back(*keeper);
    }
    // get the visible parts and sort them from large to small from the end point of the pass (the point of shooting)
    std::vector<Line> visibleParts = Field::getVisiblePartsOfGoal(true, pass.endPos, virtualBots,
            Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
    std::sort(visibleParts.begin(), visibleParts.end(),
            [](const Line &a, const Line &b) {
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
double DefencePositionCoach::penaltyForBlocks(const PossiblePass& pass, const std::vector<Vector2>& decidedBlocks) {
    // we half the score for every robot that blocks the pass
    // can be made to also take 'potential intercepts' into account better
    int amountOfBlocks = 0;
    double obstacleFactor = 0.5;
    for (const auto& block : decidedBlocks) {
        if (util::distanceToLineWithEnds(block, pass.startPos, pass.endPos)
                <= (Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS())) {
            amountOfBlocks ++;
        }
    }
    return pow(obstacleFactor, amountOfBlocks);
}
/// penalize points that are far away to pass
double DefencePositionCoach::penaltyForPassDist(PossiblePass pass) {
    // between goodUntilPassDist and impossiblePassDist we define a linear line that we multiply by the value of the distance with it.
    // e.g. at the halfway point between the two points the penalty is factor 0.5
    double goodUntilPassDist = 6.0;
    double impossiblePassDist = 15.0;
    if (pass.distance() > goodUntilPassDist) {
        return (1 - (pass.distance() - goodUntilPassDist)/(impossiblePassDist - goodUntilPassDist));
    }
    return 1.0;
}
std::vector<roboteam_msgs::WorldRobot> DefencePositionCoach::createVirtualBots(const std::vector<Vector2>& decidedBlocks) {
    std::vector<roboteam_msgs::WorldRobot> virtualBots;
    for (const auto& blockPos : decidedBlocks) {
        roboteam_msgs::WorldRobot bot;
        bot.id = - 1;
        bot.pos = blockPos;
        virtualBots.push_back(bot);
    }
    return virtualBots;
}
std::vector<std::pair<DefencePositionCoach::PossiblePass, double>> DefencePositionCoach::createPassesAndDanger(
        const std::vector<roboteam_msgs::WorldRobot>& bots, const std::vector<Vector2>& decidedBlocks) {
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
std::shared_ptr<DefencePositionCoach::DefenderBot> DefencePositionCoach::createBlockToGoal(const PossiblePass& pass, double aggressionFactor,const Line& openGoalSegment) {
    // get the blockLine segment from the ending position of the pass
    auto blockLine=getBlockLineSegment(openGoalSegment,pass.endPos);
    if (blockLine){
        DefenderBot bot;
        bot.type=botType::BLOCKTOGOAL;
        bot.blockFromID=pass.toBot.id;
        bot.targetPos=getPosOnLine(*blockLine,aggressionFactor);
        bot.orientation=getOrientation(*blockLine);
        return std::make_shared<DefenderBot>(bot);
    }
    return nullptr;
}
std::shared_ptr<DefencePositionCoach::DefenderBot> DefencePositionCoach::createBlockOnLine(const PossiblePass& pass, const Line& goalSegment) {
    // get the blockPosition on the goal line
    auto blockPos=blockOnDefenseLine(goalSegment,pass.endPos);
    if (blockPos){
        DefenderBot bot;
        bot.type=botType::BLOCKONLINE;
        bot.blockFromID=pass.toBot.id;
        bot.targetPos=*blockPos;
        //draw a line for easy orientation computation
        Line line=std::make_pair(pass.endPos,*blockPos);
        bot.orientation=getOrientation(line);
        return std::make_shared<DefenderBot>(bot);
    }
    return nullptr;
}
}//coach
}//ai
}//rtt