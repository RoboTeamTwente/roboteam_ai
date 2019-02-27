//
// Created by rolf on 18-2-19.
//

#include "DefensiveCoach.h"
#include "Field.h"
#include "../control/ControlUtils.h"

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
    auto field = Field::get_field();
    Vector2 goalLinePos1(- field.field_length*0.5, field.left_penalty_line.begin.y - margin);
    Vector2 goalLinePos2(- field.field_length*0.5, field.left_penalty_line.end.y + margin);
    Vector2 cornerPos1 = Vector2(margin, - margin) + field.left_penalty_line.begin;
    Vector2 cornerPos2 = Vector2(margin, margin) + field.left_penalty_line.end;
    if (field.left_penalty_line.begin.y > field.left_penalty_line.end.y) {
        goalLinePos1 = Vector2(- field.field_length*0.5, field.left_penalty_line.begin.y + margin);
        goalLinePos2 = Vector2(- field.field_length*0.5, field.left_penalty_line.end.y - margin);
        cornerPos1 = Vector2(margin, margin) + field.left_penalty_line.begin;
        cornerPos2 = Vector2(margin, - margin) + field.left_penalty_line.end;
    }
    // if it intersects with defense area return that,
    Vector2 intersectPos;
    if (util::lineSegmentsIntersect(point, FurthestBlock, goalLinePos1, cornerPos1)) {
        intersectPos = util::twoLineIntersection(point, FurthestBlock, goalLinePos1, cornerPos1);
    }
    else if (util::lineSegmentsIntersect(point, FurthestBlock, cornerPos1, cornerPos2)) {
        intersectPos = util::twoLineIntersection(point, FurthestBlock, cornerPos1, cornerPos2);
    }
    else if (util::lineSegmentsIntersect(point, FurthestBlock, cornerPos2, goalLinePos2)) {
        intersectPos = util::twoLineIntersection(point, FurthestBlock, cornerPos2, goalLinePos2);
    }
    else {
        // return the original line
        std::pair<Vector2, Vector2> line = std::make_pair(startPos, FurthestBlock);
        std::shared_ptr<std::pair<Vector2, Vector2>> segment = std::make_shared<std::pair<Vector2, Vector2>>(line);
        return segment;
    }
    std::pair<Vector2, Vector2> line = std::make_pair(startPos, intersectPos);
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

std::vector<std::pair<Vector2, Vector2>> DefensiveCoach::getWholeBlockSegments(std::vector<Vector2> points) {
    std::vector<std::pair<Vector2, Vector2>> segments;
    std::pair<Vector2, Vector2> goalsides = Field::getGoalSides(true);
    for (Vector2 point : points) {
        auto segment = getBlockLineSegment(goalsides, point, Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
        if (segment) {
            segments.push_back(*segment);
        }
    }
    return segments;
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
    if (theirBallBotId!=-1){
        auto botPos = std::find_if(bots.begin(),bots.end(),[theirBallBotId](roboteam_msgs::WorldRobot bot){return bot.id==theirBallBotId;});
        if (botPos!=bots.end()){
            bots.erase(botPos);
        }
    }
    while (decidedBlocks.size()!=amount&&!bots.empty()) {
        std::vector<std::pair<PossiblePass, double>> passWithScore;
        for (auto bot : bots) {
            PossiblePass pass(bot, World::getBall()->pos);
            double danger = scorePossiblePass2(pass,decidedBlocks);
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
            auto botPos = std::find_if(bots.begin(),bots.end(),[bestPass](roboteam_msgs::WorldRobot b){return b.id==bestPass.first.toBot.id;});
            if (botPos!=bots.end()){
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
    return line.second + (line.first- line.second)*aggressionFactor;
}
double DefensiveCoach::scorePossiblePass2(PossiblePass pass, std::vector<Vector2> decidedBlocks){
    double score = 1.0;
    double obstacleFactor = 0.5;
    int amountOfBlocks=0;
    for (auto block : decidedBlocks){
        if (util::distanceToLineWithEnds(block,pass.startPos,pass.endPos)<=(Constants::ROBOT_RADIUS()+Constants::BALL_RADIUS())){
            amountOfBlocks++;
        }
    }

    score = score*pow(obstacleFactor, amountOfBlocks);
    double goodUntilPassDist = 4.0;
    double impossiblePassDist = 10.0;
    if (pass.distance() > goodUntilPassDist) {
        score = score*(1 - (pass.distance() - goodUntilPassDist)/(impossiblePassDist - goodUntilPassDist));
    }
    return score;
}
}
}
}