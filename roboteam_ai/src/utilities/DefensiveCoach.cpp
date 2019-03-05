//
// Created by rolf on 18-2-19.
//

#include "DefensiveCoach.h"
#include "Field.h"
#include "../control/ControlUtils.h"
#include "RobotDealer.h"

/// This is a class that returns the positions we want our defenders to be at for all defenders
namespace rtt {
namespace ai {
namespace coach {
using util = control::ControlUtils;

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

///returns all passes that are not obstructed
std::vector<DefensiveCoach::PossiblePass> DefensiveCoach::getPossiblePassesThem() {
    std::vector<PossiblePass> possiblePasses;
    //first find if one of their bots has the ball and if so which bot
    int theirBotWithBall = World::whichBotHasBall(false);
    std::shared_ptr<roboteam_msgs::WorldRobot> passBot = World::getRobotForId(theirBotWithBall, false);
    if (theirBotWithBall == - 1 || ! passBot) {
        return possiblePasses;
    }
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
double DefensiveCoach::scorePossiblePass(DefensiveCoach::PossiblePass pass) {
    double score = 1.0;
    double obstacleFactor = 0.5;
    score = score*pow(obstacleFactor, (double) pass.amountOfBlockers());
    double goodUntilPassDist = 4.0;
    double impossiblePassDist = 10.0;
    if (pass.distance() > goodUntilPassDist) {
        score = score*(1 - (pass.distance() - goodUntilPassDist)/(impossiblePassDist - goodUntilPassDist));
    }
    return score;
}
std::shared_ptr<std::pair<Vector2, Vector2>> DefensiveCoach::getBlockLineSegment(
        std::pair<Vector2, Vector2> openGoalSegment, Vector2 point, double collisionRadius) {
    double margin = collisionRadius;
    //compute the bisector
    Vector2 lineToSideOne = openGoalSegment.first - point;
    Vector2 lineToSideTwo = (openGoalSegment.second - point).stretchToLength(lineToSideOne.length());
    Vector2 startPos = point + (lineToSideOne + lineToSideTwo).stretchToLength(collisionRadius);
    if (Field::pointIsInDefenceArea(startPos, true, margin)) {
        return nullptr;
    }
    Vector2 endPos = point + (lineToSideOne + lineToSideTwo)*0.5;// this defines the line on which the bisector lies.
    double theta = lineToSideOne.angle() - (endPos - point).angle();
    double collisionDist = collisionRadius/sin(theta);
    Vector2 FurthestBlock = point + Vector2(collisionDist, 0).rotate((endPos - point).angle());
    //check intersections with defense area
    std::shared_ptr<Vector2> intersectPos=Field::lineIntersectsWithDefenceArea(true,point,FurthestBlock,margin);
    if (!intersectPos) {
        // return the original line
        std::pair<Vector2, Vector2> line = std::make_pair(startPos, FurthestBlock);
        std::shared_ptr<std::pair<Vector2, Vector2>> segment = std::make_shared<std::pair<Vector2, Vector2>>(line);
        return segment;
    }
    std::pair<Vector2, Vector2> line = std::make_pair(startPos, *intersectPos);
    std::shared_ptr<std::pair<Vector2, Vector2>> segment = std::make_shared<std::pair<Vector2, Vector2>>(line);
    return segment;

}
std::shared_ptr<std::pair<Vector2, Vector2>> DefensiveCoach::blockBall() {
    if (World::getBall()) {
        auto goalSides = Field::getGoalSides(true);
        return getBlockLineSegment(goalSides, World::getBall()->pos,
                Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
    }
    return nullptr;
}

std::vector<std::pair<Vector2, Vector2>> DefensiveCoach::decideDefenderLocations(int amount) {
    std::vector<std::pair<Vector2, Vector2>> decidedLines;
    if (amount <= 0) {
        ROS_ERROR("Can't assign 0 or less Defender locations!!");
        return decidedLines;
    }
    // we decide the lines on which we want the defenders to be
    //most importantly; make sure the ball can't be kicked into the goal directly
    auto priorityPos = blockBall();
    if (priorityPos) {
        decidedLines.push_back(*priorityPos);
    }
    if (decidedLines.size() == amount) {
        return decidedLines;
    }
    // now we look for passes and cover off the good positions from there
    std::pair<Vector2, Vector2> goalSides = Field::getGoalSides(true);
    int theirBallBotId = World::whichBotHasBall(false);
    auto bots = World::get_world().them;
    std::vector<std::pair<PossiblePass, double>> passWithScore;
    for (auto bot : bots) {
        if (bot.id != theirBallBotId) {
            PossiblePass pass(bot, World::getBall()->pos);
            double danger = scorePossiblePass(pass);
            std::pair<PossiblePass, double> passPair = std::make_pair(pass, danger);
            passWithScore.push_back(passPair);
        }
    }
    //order passes from most dangerous to least dangerous
    std::sort(passWithScore.begin(), passWithScore.end(),
            [](std::pair<PossiblePass, double> &left, std::pair<PossiblePass, double> &right) {
              if (left.second > right.second) { return true; }
              else if (right.second > left.second) { return false; }
              else
                  return (Field::getTotalGoalAngle(true, left.first.endPos)
                          > Field::getTotalGoalAngle(true, right.first.endPos));
            });
    for (const auto &bestPass: passWithScore) {
        auto segment = getBlockLineSegment(goalSides, bestPass.first.endPos,
                Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
        if (segment) {
            decidedLines.push_back(*segment);
            if (decidedLines.size() == amount) {
                return decidedLines;
            }
        }
    }
    return decidedLines;
}

std::vector<Vector2> DefensiveCoach::decideDefenderLocations2(int amount, double aggressionFactor) {
    std::vector<Vector2> decidedBlocks;
    if (amount <= 0) {
        ROS_ERROR("Can't assign 0 or less Defender locations!!");
        return decidedBlocks;
    }
    // we decide the lines on which we want the defenders to be
    //most importantly; make sure the ball can't be kicked into the goal directly
    auto priorityPos = blockBall();
    if (priorityPos) {
        decidedBlocks.push_back(getPos(*priorityPos, aggressionFactor));
    }
    if (decidedBlocks.size() == amount) {
        return decidedBlocks;
    }
    // now we look for passes and cover off the good positions from there
    // we assign a position to one robot and then recompute the passes again

    std::pair<Vector2, Vector2> goalSides = Field::getGoalSides(true);
    int theirBallBotId = World::whichBotHasBall(false);
    auto bots = World::get_world().them;
    if (theirBallBotId != - 1) {
        auto botPos = std::find_if(bots.begin(), bots.end(),
                [theirBallBotId](roboteam_msgs::WorldRobot bot) { return bot.id == theirBallBotId; });
        if (botPos != bots.end()) {
            bots.erase(botPos);
        }
    }
    while (decidedBlocks.size() != amount && ! bots.empty()) {
        std::vector<std::pair<PossiblePass, double>> passWithScore;
        for (auto bot : bots) {
            PossiblePass pass(bot, World::getBall()->pos);
            double danger = scorePossiblePass3(pass, decidedBlocks);
            std::pair<PossiblePass, double> passPair = std::make_pair(pass, danger);
            passWithScore.push_back(passPair);
        }
        //order passes from most dangerous to least dangerous
        std::sort(passWithScore.begin(), passWithScore.end(),
                [](std::pair<PossiblePass, double> &left, std::pair<PossiblePass, double> &right) {
                  return left.second > right.second;
                });

        for (const auto &bestPass: passWithScore) {
            auto segment = getBlockLineSegment(goalSides, bestPass.first.endPos,
                    Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
            // we erase the robot from the positions to be searched
            auto botPos = std::find_if(bots.begin(), bots.end(),
                    [bestPass](roboteam_msgs::WorldRobot b) { return b.id == bestPass.first.toBot.id; });
            if (botPos != bots.end()) {
                bots.erase(botPos);
            }
            if (segment) {
                decidedBlocks.push_back(getPos(*segment, aggressionFactor));
                if (decidedBlocks.size() == amount) {
                    return decidedBlocks;
                }
                break;
            }

        }
    }
    return decidedBlocks;
}

Vector2 DefensiveCoach::getPos(std::pair<Vector2, Vector2> line, double aggressionFactor) {
    if (aggressionFactor < 0) {
        return line.second;
    }
    else if (aggressionFactor > 1) {
        return line.first;
    }
    return line.second + (line.first - line.second)*aggressionFactor;
}

double DefensiveCoach::scorePossiblePass2(PossiblePass pass, std::vector<Vector2> decidedBlocks) {
    double score = 1.0;

    std::vector<roboteam_msgs::WorldRobot> virtualBots;
    auto keeper = World::getRobotForId(robotDealer::RobotDealer::getKeeperID(), true);
    if (keeper) {
        virtualBots.push_back(*keeper);
    }
    for (auto blockPos : decidedBlocks) {
        roboteam_msgs::WorldRobot bot;
        bot.id = - 1;
        bot.pos = blockPos;
        virtualBots.push_back(bot);
    }
    std::vector<std::pair<Vector2, Vector2>> visibleParts = Field::getVisiblePartsOfGoal(true, pass.endPos, virtualBots,
            Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
    std::sort(visibleParts.begin(), visibleParts.end(),
            [](const std::pair<Vector2, Vector2> &a, const std::pair<Vector2, Vector2> &b) {
              return abs(a.second.y - a.first.y) > abs(b.second.y - b.first.y);
            });
    double largestOpenGoalAngle;
    if (visibleParts.empty()) {
        largestOpenGoalAngle = Field::getTotalGoalAngle(true, pass.endPos)*0.05;
    }
    else {
        double angleOne = (visibleParts[0].first - pass.endPos).angle();
        double angleTwo = (visibleParts[0].second - pass.endPos).angle();
        largestOpenGoalAngle = control::ControlUtils::angleDifference(control::ControlUtils::constrainAngle(angleOne),
                control::ControlUtils::constrainAngle(angleTwo));
    }

    score = score*largestOpenGoalAngle;

    // penalties
    int amountOfBlocks = 0;
    double obstacleFactor = 0.5;
    for (auto block : decidedBlocks) {
        if (util::distanceToLineWithEnds(block, pass.startPos, pass.endPos)
                <= (Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS())) {
            amountOfBlocks ++;
        }
    }
    score = score*pow(obstacleFactor, amountOfBlocks);
    double goodUntilPassDist = 6.0;
    double impossiblePassDist = 15.0;
    if (pass.distance() > goodUntilPassDist) {
        score = score*(1 - (pass.distance() - goodUntilPassDist)/(impossiblePassDist - goodUntilPassDist));
    }
    return score;
}

double DefensiveCoach::scorePossiblePass3(PossiblePass pass, std::vector<Vector2> decidedBlocks) {
    double score = 0.0;
    int amountOfBlocks = 0;
    double obstacleFactor = 0.5;
    double goodUntilPassDist = 6.0;
    double impossiblePassDist = 15.0;
    double percentageVisibleFactor = 0.2;
    double distanceFactor = 1.0;
    std::vector<roboteam_msgs::WorldRobot> virtualBots;
    for (auto blockPos : decidedBlocks) {
        roboteam_msgs::WorldRobot bot;
        bot.id = - 1;
        bot.pos = blockPos;
        virtualBots.push_back(bot);
    }
    std::vector<std::pair<Vector2, Vector2>> visibleParts = Field::getVisiblePartsOfGoal(true, pass.endPos, virtualBots,
            Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
    std::sort(visibleParts.begin(), visibleParts.end(),
            [](const std::pair<Vector2, Vector2> &a, const std::pair<Vector2, Vector2> &b) {
              return abs(a.second.y - a.first.y) > abs(b.second.y - b.first.y);
            });
    double largestOpenGoalAngle;
    if (visibleParts.empty()) {
        largestOpenGoalAngle = Field::getTotalGoalAngle(true, pass.endPos)*0.05;
    }
    else {
        double angleOne = (visibleParts[0].first - pass.endPos).angle();
        double angleTwo = (visibleParts[0].second - pass.endPos).angle();
        largestOpenGoalAngle = control::ControlUtils::angleDifference(control::ControlUtils::constrainAngle(angleOne),
                control::ControlUtils::constrainAngle(angleTwo));
    }
    std::pair<Vector2, Vector2> goalSides = Field::getGoalSides(true);
    score = score + percentageVisibleFactor*largestOpenGoalAngle/Field::getTotalGoalAngle(true, pass.endPos);
    std::cout << score << " After visibility" << std::endl;
    score = score
            + distanceFactor/((pass.endPos - goalSides.first).length() + (pass.endPos - goalSides.second).length());
    std::cout << score << " after total" << std::endl;

    // penalties

    for (auto block : decidedBlocks) {
        if (util::distanceToLineWithEnds(block, pass.startPos, pass.endPos)
                <= (Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS())) {
            amountOfBlocks ++;
        }
    }
    score = score*pow(obstacleFactor, amountOfBlocks);;
    if (pass.distance() > goodUntilPassDist) {
        score = score*(1 - (pass.distance() - goodUntilPassDist)/(impossiblePassDist - goodUntilPassDist));
    }
    return score;
}

std::vector<Vector2> DefensiveCoach::doubleBlockOnDefenseLine(
        std::pair<Vector2, Vector2> openGoalSegment, Vector2 point) {
    std::vector<Vector2> blockPositions;
    double margin = 0.1;
    double collisionRadius = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
    //compute the bisector
    Vector2 lineToSideOne = openGoalSegment.first - point;
    Vector2 lineToSideTwo = (openGoalSegment.second - point).stretchToLength(lineToSideOne.length());
    Vector2 startPos = point + (lineToSideOne + lineToSideTwo).stretchToLength(collisionRadius);
    if (Field::pointIsInDefenceArea(startPos, true, margin)) {
        return blockPositions;
    }
    Vector2 endPos = point + (lineToSideOne + lineToSideTwo)*0.5;// this defines the line on which the bisector lies.
    // now compute intersection with the defense area
    double theta = lineToSideOne.angle() - (endPos - startPos).angle();
    double collisionDist = collisionRadius/sin(theta);
    Vector2 FurthestBlock = point + Vector2(collisionDist, 0).rotate((endPos - startPos).angle());
    // if it intersects with defense area return that,
    std::shared_ptr<Vector2> intersectPos;
    intersectPos=Field::lineIntersectsWithDefenceArea(true,startPos,FurthestBlock,margin);
    if(!intersectPos) {
        // try double blocking;
        Vector2 posOne, posTwo;
        collisionDist = (2*Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS())/sin(theta);
        FurthestBlock = point + Vector2(collisionDist, 0).rotate((endPos - startPos).angle());
        std::shared_ptr<Vector2> centerPos=Field::lineIntersectsWithDefenceArea(true,startPos,FurthestBlock,margin);
        if (centerPos){
            posOne =*centerPos + Vector2(Constants::ROBOT_RADIUS(), 0).rotate((endPos - startPos).angle() + M_PI_2);
            posTwo =*centerPos + Vector2(Constants::ROBOT_RADIUS(), 0).rotate((endPos - startPos).angle() - M_PI_2);
        }
        else {
            posOne =FurthestBlock + Vector2(Constants::ROBOT_RADIUS(), 0).rotate((endPos - startPos).angle() + M_PI_2);
            posTwo =FurthestBlock + Vector2(Constants::ROBOT_RADIUS(), 0).rotate((endPos - startPos).angle() - M_PI_2);
        }
        std::shared_ptr<Vector2> intersectPosOne=Field::lineIntersectsWithDefenceArea(true,point,posOne,margin);
        std::shared_ptr<Vector2> intersectPosTwo=Field::lineIntersectsWithDefenceArea(true,point,posTwo,margin);
        if (intersectPosOne){posOne=*intersectPosOne;}
        if (intersectPosTwo){posTwo=*intersectPosTwo;}
        blockPositions.push_back(posOne);
        blockPositions.push_back(posTwo);
        return blockPositions;
    }
    // we have only one intersection
    blockPositions.push_back(*intersectPos);
    return blockPositions;

}

std::shared_ptr<Vector2> DefensiveCoach::blockOnDefenseLine(std::pair<Vector2, Vector2> openGoalSegment,
        Vector2 point) {
    double margin = 0.1;
    double collisionRadius = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
    //compute the bisector
    Vector2 lineToSideOne = openGoalSegment.first - point;
    Vector2 lineToSideTwo = (openGoalSegment.second - point).stretchToLength(lineToSideOne.length());
    Vector2 startPos = point + (lineToSideOne + lineToSideTwo).stretchToLength(collisionRadius);
    if (Field::pointIsInDefenceArea(startPos, true, margin)) {
        return nullptr;
    }
    Vector2 endPos = point + (lineToSideOne + lineToSideTwo)*0.5;// this defines the line on which the bisector lies.
    // now compute intersection with the defense area
    return Field::lineIntersectsWithDefenceArea(true,point,endPos,margin);
}

std::vector<Vector2> DefensiveCoach::decideDefendersOnDefenseLine(int amount) {
    std::vector<Vector2> decidedBlocks;
    if (amount <= 0) {
        ROS_ERROR("Can't assign 0 or less Defender locations!!");
        return decidedBlocks;
    }
    // we decide the lines on which we want the defenders to be
    //most importantly; make sure the ball can't be kicked into the goal directly; we put one or more robots as a direct block;

    std::pair<Vector2, Vector2> goalSides = Field::getGoalSides(true);
    if (World::getBall()) {
        Vector2 mostDangerousPos = World::getBall()->pos; //TODO: update for pass detection
        decidedBlocks = doubleBlockOnDefenseLine(goalSides, mostDangerousPos);
        if (amount == 1 && decidedBlocks.size() == 2) {
            auto segment = getBlockLineSegment(goalSides, mostDangerousPos,
                    Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
            if (segment) {
                return {getPos(*segment, 0)};
            }
            return {};
        }
    }
    if (amount == decidedBlocks.size()) {
        return decidedBlocks;
    }
    // now we look for passes and cover off the good positions from there
    // we assign a position to one robot and then recompute the passes again

    int theirBallBotId = World::whichBotHasBall(false);
    auto bots = World::get_world().them;
    if (theirBallBotId != - 1) {
        auto botPos = std::find_if(bots.begin(), bots.end(),
                [theirBallBotId](roboteam_msgs::WorldRobot bot) { return bot.id == theirBallBotId; });
        if (botPos != bots.end()) {
            bots.erase(botPos);
        }
    }
    while (decidedBlocks.size() != amount && ! bots.empty()) {
        std::vector<std::pair<PossiblePass, double>> passWithScore;
        for (auto bot : bots) {
            PossiblePass pass(bot, World::getBall()->pos);
            double danger = scorePossiblePass2(pass, decidedBlocks);
            std::pair<PossiblePass, double> passPair = std::make_pair(pass, danger);
            passWithScore.push_back(passPair);
        }
        //order passes from most dangerous to least dangerous
        std::sort(passWithScore.begin(), passWithScore.end(),
                [](std::pair<PossiblePass, double> &left, std::pair<PossiblePass, double> &right) {
                  return left.second > right.second;
                });

        std::vector<roboteam_msgs::WorldRobot> virtualBots;
        for (auto blockPos : decidedBlocks) {
            roboteam_msgs::WorldRobot bot;
            bot.id = - 1;
            bot.pos = blockPos;
            virtualBots.push_back(bot);
        }

        for (const auto &bestPass: passWithScore) {
            std::vector<std::pair<Vector2, Vector2>> visibleParts = Field::getVisiblePartsOfGoal(true,
                    bestPass.first.endPos, virtualBots,
                    Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
            std::sort(visibleParts.begin(), visibleParts.end(),
                    [](const std::pair<Vector2, Vector2> &a, const std::pair<Vector2, Vector2> &b) {
                      return abs(a.second.y - a.first.y) > abs(b.second.y - b.first.y);
                    });
            //auto blockPoint =blockOnDefenseLine(goalSides, bestPass.first.endPos);
            std::shared_ptr<Vector2> blockPoint;
            if (! visibleParts.empty()) {
                blockPoint = blockOnDefenseLine(visibleParts[0], bestPass.first.endPos);
            }
            // we erase the robot from the positions to be searched
            auto botPos = std::find_if(bots.begin(), bots.end(),
                    [bestPass](roboteam_msgs::WorldRobot b) { return b.id == bestPass.first.toBot.id; });
            if (botPos != bots.end()) {
                bots.erase(botPos);
            }
            if (blockPoint) {
                decidedBlocks.push_back(*blockPoint);
                if (decidedBlocks.size() == amount) {
                    return decidedBlocks;
                }
                break;
            }

        }
    }
    return decidedBlocks;
}
}
}
}