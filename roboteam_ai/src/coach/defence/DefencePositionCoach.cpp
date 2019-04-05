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
/// places two robots as a block such that they are closest to the goal but block
std::vector<Vector2> DefencePositionCoach::doubleBlockOnDefenseLine(
        const Line &openGoalSegment, const Vector2 &point) {
    std::vector<Vector2> blockPositions;
    double margin = 0.15;
    double collisionRadius = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();

    Vector2 FurthestBlock = getBlockPoint(openGoalSegment, point, collisionRadius);
    Vector2 startPos =
            point + (FurthestBlock - point).stretchToLength(collisionRadius);//start should be out of collision radius
    // if the starting position is in the defence area you cannot 'squeeze a robot in between the position and the defence area
    if (world::field->pointIsInDefenceArea(startPos, true, margin)) {
        return blockPositions;
    }
    std::shared_ptr<Vector2> intersectPos = world::field->lineIntersectsWithDefenceArea(true, startPos, FurthestBlock,
            margin);
    if (! intersectPos) {
        // else try double blocking ; intersect radius goes up to 2*robotRadius+ball radius
        Vector2 blockPos;
        collisionRadius = 2*Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
        FurthestBlock = getBlockPoint(openGoalSegment, point, collisionRadius);
        std::shared_ptr<Vector2> intersectDoublePos = world::field->lineIntersectsWithDefenceArea(true, startPos,
                FurthestBlock, margin);
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
            std::shared_ptr<Vector2> intersection = world::field->lineIntersectsWithDefenceArea(true, point, blockPos,
                    margin);
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
/*
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
*/
/// gets the furthest position at which an obstacle will block the entire Angle
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
    world::WorldData simulatedWorld=world::world->getWorld();
    while (defenders.size() != amount) {
        std::vector<PossiblePass> passes = createPassesSortedByDanger(simulatedWorld);
        if (! passes.empty()) {
            //first we try to create a direct block to the goal
            auto defender = createBlockToGoal(passes[0], 0.0, simulatedWorld);
            if (defender) {
                defenders.push_back(*defender);
                continue;
            }
            // if that doesn't work try to directly intercept the pass
            //defender = createBlockPass(passes[0], simulatedWorld);
        }
    }
    return defenders;
}

std::vector<PossiblePass> DefencePositionCoach::createPassesSortedByDanger(
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
std::shared_ptr<DefencePositionCoach::DefenderBot> DefencePositionCoach::createBlockToGoal(const PossiblePass &pass,
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
std::shared_ptr<DefencePositionCoach::DefenderBot> DefencePositionCoach::createBlockOnLine(const PossiblePass &pass,
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
}//coach
}//ai
}//rtt