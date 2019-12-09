
//
// Created by rolf on 18-2-19.
//

#include "coach/defence/DefencePositionCoach.h"

#include <utility>
#include "world/Field.h"
#include "control/ControlUtils.h"
#include "utilities/RobotDealer.h"

/// This is a class that computes useful lines and positions for computing defender positions
namespace rtt::ai::coach {

    using util = control::ControlUtils;

    DefencePositionCoach g_defensivePositionCoach;

    bool DefenderBot::validPosition(const world::WorldData &world) const noexcept {
        return std::none_of(world.us.begin(), world.us.end(),
                            [&](auto const &bot) {
                                return (bot->pos - targetPos).length() < 2 * Constants::ROBOT_RADIUS();
                            });
    }

    world::Robot::RobotPtr DefenderBot::toRobot() const noexcept {
        world::Robot::RobotPtr robot = std::make_shared<world::Robot>();
        robot->id = -1;
        robot->pos = targetPos;
        robot->angle = orientation;
        return robot;
    }


    std::optional<DefenderBot> DefencePositionCoach::blockMostDangerousPos() const noexcept {
        //block the most dangerous position to the goal completely if possible.
        // check if it is possible to find a position that is legal
        auto crucialBlock = defender_pos::blockBallLine(simulatedWorld);
        if (crucialBlock) { // if so, create a defender
            return defender_pos::createBlockBall(*crucialBlock);
        }
        return std::nullopt;
    }

    std::optional<DefenderBot> DefencePositionCoach::blockPass(const PossiblePass &pass) const noexcept {
        // this function tries multiple ways to block a pass. Returns true if it succeeded, false if it fails.
        // first try blocking the goal vision of the robot being passed to
        auto blockLine = defender_pos::blockToGoalLine(pass, simulatedWorld);
        if (blockLine) {
            // try to find a position on the line that is valid
            auto aggressionFactor = defender_pos::pickNewPosition(*blockLine, simulatedWorld);
            if (aggressionFactor) {
                return defender_pos::createBlockToGoal(pass, *aggressionFactor, *blockLine);
            }
            // try putting it on the defence Line instead (as the robot is very likely far away
            double fieldWidth = world::field->get_field().field_width();

            // Floating point errors sigh (hence the -0.0001)
            Vector2 bottomLine(defender_pos::maxX() - 0.0001, -fieldWidth * 0.5);
            Vector2 topLine(defender_pos::maxX() - 0.0001, fieldWidth * 0.5);
            Vector2 intersectPos = control::ControlUtils::twoLineIntersection(blockLine->first,
                                                                              blockLine->second, bottomLine,
                                                                              topLine);
            if (defender_pos::validNewPosition(intersectPos, simulatedWorld)) {
                return defender_pos::createBlockToGoal(pass, intersectPos, *blockLine);
            }
        }
        // then try blocking on the defense line (closer to the line) if that is possible
        auto blockPos = defender_pos::blockOnDefenseAreaLine(pass, simulatedWorld);
        if (blockPos) {
            if (defender_pos::validNewPosition(*blockPos, simulatedWorld)) {
                return defender_pos::createBlockOnLine(pass, *blockPos);
            }
        }
        // then try to intercept the pass, we try to find a spot along the pass where we can stand
        auto passBlock = defender_pos::pickNewPosition(pass, simulatedWorld);
        if (passBlock) {
            return defender_pos::createBlockPass(pass, *passBlock);
        }
        return std::nullopt;
    }

// if we add a defender we want to both add it to simulation and our stored defender array for return
    void DefencePositionCoach::addDefender(const DefenderBot& defender) noexcept {
        simulatedWorld.us.emplace_back(std::move(defender.toRobot()));
        defenders.emplace(defender.id, defender);
    }

    std::map<int, DefenderBot> DefencePositionCoach::decidePositions(const std::map<int, DefenderBot> &lockedDefenders,
                                                                     const std::set<int> &oldFreeBots) noexcept {
        std::map<int, DefenderBot> oldDefenders = defenders;
        defenders.clear(); // we are recomputing the positions again
        size_t defenderAmount = lockedDefenders.size() + oldFreeBots.size();
        if (defenderAmount <= 0) {
            return defenders;
        } // we don't actually need to calculate now.
        simulatedWorld = defender_pos::setupSimulatedWorld();
        // handle all the locked robots
        auto[blockedMostDangerousPos, lockedCount, freeRobots] = decideLockedPositions(lockedDefenders, oldFreeBots);

        // now we only have free robots left;
        if (!blockedMostDangerousPos && defenders.size() < defenderAmount) {
            auto bot = blockMostDangerousPos(); //first we handle the most dangerous position first
            if (bot) {
                addDefender(*bot);
            }
        }
        // for the remainder we look at the possiblePasses and block the most dangerous bots
        std::vector<PossiblePass> passes = defender_pos::createPassesSortedByDanger(simulatedWorld);
        while ((defenders.size()) != defenderAmount && !passes.empty()) {
            auto foundNewDefender = blockPass(
                    passes[0]); // we try to cover the most dangerous pass in multiple ways
            // if we find a defender we need to recalculate the danger of our passes to reflect the new robot.
            if (!foundNewDefender) {
                // if we cannot find a way to cover it, we remove the attacker from the simulated world (otherwise we get 'stuck')
                defender_pos::removeBotFromWorld(simulatedWorld, passes[0].toBot.id, false);
                // this should pretty much never happen.
                std::cerr << "Pass to robot" << passes[0].toBot.id << " removed in defensiveCoach!"
                          << std::endl;
            } else {
                addDefender(*foundNewDefender);
            }
            passes = defender_pos::createPassesSortedByDanger(
                    simulatedWorld); //recalculate the danger after the new position
        }
        // TODO: find a procedure if we cannot really cover all of the robots off, e.g. default positions or such
        assignIDs(lockedCount, freeRobots,
                  oldDefenders); // divide the ID's of the last robots over the remaining available ID's.
        return defenders;
    }

    std::tuple<bool, int, std::set<int>>
    DefencePositionCoach::decideLockedPositions(const std::map<int, rtt::ai::coach::DefenderBot> &lockedDefenders,
                                                std::set<int> freeRobots) noexcept {
        bool blockedMostDangerousPos = false;
        int lockedCount = 0;
        std::vector<PossiblePass> passes = defender_pos::createPassesSortedByDanger(simulatedWorld);
        for (const auto &[id, lockedDefender] : lockedDefenders) {
            bool replacedDefender = false;
            if (lockedDefender.type != BLOCKBALL) {
                for (const auto &pass : passes) {
                    if (pass.toBot.id == lockedDefender.blockFromID) {
                        setNewDefender(lockedDefender, pass, lockedCount, replacedDefender);
                        break;
                    }
                }
            } else {
                // we need special handling for if the robot is blocking the most dangerous position
                mostDangerousBlocking(lockedDefender, blockedMostDangerousPos, lockedCount, replacedDefender);
            }
            if (!replacedDefender) {
                freeRobots.emplace(
                        lockedDefender.id);// if we somehow cannot cover this robot anymore, we set it to free
            }
        }
        return std::make_tuple(blockedMostDangerousPos, lockedCount, freeRobots);
    }

    void
    DefencePositionCoach::setNewDefender(const DefenderBot &lockedDefender, const PossiblePass &pass, int &lockedCount,
                                         bool &replacedDefender) {
        auto newDefender = blockPass(pass);
        if (!newDefender) {
            return;
        }
        lockedCount++;
        newDefender->id = lockedDefender.id;
        newDefender->coveredCount = lockedDefender.coveredCount + 1;
        addDefender(*newDefender);
        replacedDefender = true;
    }

    void DefencePositionCoach::mostDangerousBlocking(const DefenderBot &lockedDefender,
                                                     bool &blockedMostDangerousPos,
                                                     int &lockedCount, bool &replacedDefender) {
        auto newDefender = blockMostDangerousPos();
        if (!newDefender) {
            return;
        }
        lockedCount++;
        newDefender->id = lockedDefender.id;
        newDefender->coveredCount = lockedDefender.coveredCount + 1;
        addDefender(*newDefender);
        blockedMostDangerousPos = true;
        replacedDefender = true;

    }

// the following algorithm takes the closest robot for each available defender to decide which robot goes where.
// Since the points are ordered on priority from the above algorithm the most important points come first
// It might be better to use an algorithm that is more complicated (e.g. hungarian) but then we might need some kind of system which gives the first points more 'priority'
    void DefencePositionCoach::assignIDs(int lockedCount, std::set<int> freeRobotIDs,
                                         const std::map<int, DefenderBot> &oldDefenders) noexcept {
        for (size_t j = lockedCount; j < defenders.size(); ++j) {
            int closestId = -1;
            auto closestDist = 9e9;
            for (int botId : freeRobotIDs) {
                auto bot = world::world->getRobotForId(botId, true);
                if (bot) {
                    if ((defenders[j].targetPos - bot->pos).length() < closestDist) {
                        closestId = botId;
                        closestDist = (defenders[j].targetPos - bot->pos).length();
                    }
                }
            }
            if (closestId != -1) {
                defenders[j].id = closestId;
                freeRobotIDs.erase(std::find(freeRobotIDs.begin(), freeRobotIDs.end(), closestId));
                for (const auto &[oldId, oldDefender] : oldDefenders) {
                    // if the robot is still covering the same target.
                    if (oldId == closestId && oldDefender.blockFromID == defenders[j].blockFromID) {
                        defenders[j].coveredCount = oldDefender.coveredCount + 1;
                        break;
                    }
                }
            }
        }
    }


    namespace defender_pos {
        world::WorldData setupSimulatedWorld() {
            world::WorldData sWorld = world::world->getWorld();
            sWorld.us.clear();
            sWorld = getTheirAttackers(sWorld);    // we select only the relevant robots
            return sWorld;
        }


        world::WorldData getTheirAttackers(const world::WorldData &world) {
            std::vector<world::Robot::RobotPtr> theirAttackers;
            for (auto &robot :world.them) {
                // we remove any attackers that are outside of the field or in our defence area
                if (!world::field->pointIsInDefenceArea(robot->pos, true, 0.04)
                    && world::field->pointIsInField(robot->pos, -0.1)) {
                    theirAttackers.push_back(robot);
                }
            }
            world::WorldData newWorld = world;
            newWorld.them = theirAttackers;
            return newWorld;
        }

        bool validNewPosition(const Vector2 &position, const world::WorldData &world) {
            if (position.x > maxX()) { return false; }
            double collisionRadius = DefencePositionCoach::CALCULATION_COLLISION_RADIUS;// a little smaller than 2 robot radii so that we can make solid walls still
            return std::none_of(world.us.begin(), world.us.end(),
                                [&](auto const &robot) { return (robot->pos - position).length() < collisionRadius; });
        }

        std::optional<double> pickNewPosition(const Line &line,
                                              const world::WorldData &world) {
            //search a position on the line on which we can position.
            for (int aggresionFactor = 0;
                 aggresionFactor <= DefencePositionCoach::SEARCH_POINTS; ++aggresionFactor) {
                if (validNewPosition(getPosOnLine(line, aggresionFactor /
                                                        DefencePositionCoach::SEARCH_POINTS),
                                     world)) {
                    return aggresionFactor;
                }
            }
            // the whole line is completely blocked (atleast on the points we tested)
            return std::nullopt;
        }

        std::optional<Vector2> pickNewPosition(PossiblePass pass, const world::WorldData &world) {
            std::optional<Vector2> point{};
            double segments = 30.0;
            // we pick new points on which we can defend preferably as close as possible to the middle of the pass.
            for (int j = 0; j <= segments * 0.5; ++j) {
                Vector2 forwardPosition = pass.posOnLine(0.5 + j / segments);
                if (validNewPosition(forwardPosition, world) &&
                    !world::field->pointIsInDefenceArea(forwardPosition, true,
                                                        DefencePositionCoach::DEFENSELINE_MARGIN)) {
                    return forwardPosition;
                }
                Vector2 backwardPosition = pass.posOnLine(0.5 - j / segments);
                if (validNewPosition(backwardPosition, world) &&
                    !world::field->pointIsInDefenceArea(forwardPosition, true,
                                                        DefencePositionCoach::DEFENSELINE_MARGIN)) {
                    return backwardPosition;
                }
            }
            return std::nullopt;
        }

        double maxX() {
            return world::field->get_field().field_length() / -10.0;
        }

        // pick position on the line depending on how aggressive we want to play. aggression factor 1 is very in your face, whilst 0 is as close as possible to the goal
        Vector2 getPosOnLine(const Line &line, double aggressionFactor) {
            if (aggressionFactor < 0) {
                return line.second;
            } else if (aggressionFactor > 1) {
                return line.first;
            }
            return line.second + (line.first - line.second) * aggressionFactor;
        }


        DefenderBot createBlockPass(PossiblePass const &pass,
                                    const Vector2 &blockPoint) {
            DefenderBot bot;
            bot.type = botType::BLOCKPASS;
            bot.blockFromID = pass.toBot.id;
            bot.targetPos = blockPoint;
            bot.orientation = pass.faceLine();
            return bot;
        }


// get the direction facing towards the end of the Line
        double getOrientation(const Line &line) {
            return (line.first - line.second).angle();
        }

        //computes a line segment on which the entirety of openGoalSegment is blocked as seen from point with robots with radius collissionRadius
        std::optional<Line>
        getBlockLineSegment(const Line &openGoalSegment, const Vector2 &point, double collisionRadius, double margin) {
            if (margin == -1.0) {
                margin = DefencePositionCoach::DEFENSELINE_MARGIN;
            }

            Vector2 FurthestBlock = getBlockPoint(openGoalSegment, point, collisionRadius);
            Vector2 startPos =
                    point + (FurthestBlock - point).stretchToLength(
                            collisionRadius);//start should be out of collision radius
            // if the starting position is in the defence area you cannot 'squeeze a robot in between the position and the defence area
            if (world::field->pointIsInDefenceArea(startPos, true, margin)) {
                return std::nullopt;
            }
            //check intersections with defense area and shorten line if needed
            return shortenLineForDefenseArea(startPos, FurthestBlock, margin);

        }

/// computes the intersection of the bisector of the angle to OpenGoalsegment from point and the defence area line
        std::optional<Vector2> blockOnDefenseLine(const Line &openGoalSegment,
                                                  const Vector2 &point) {
            // margin by which we shift the defence area line forwards
            double collisionRadius = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
            //compute the bisector
            Vector2 lineToSideOne = openGoalSegment.first - point;
            Vector2 lineToSideTwo = (openGoalSegment.second - point);
            Vector2 startPos = point + (lineToSideOne + lineToSideTwo).stretchToLength(collisionRadius);
            // if starting point is in the defence area there is no room for a robot to squeeze in
            if (world::field->pointIsInDefenceArea(startPos, true, DefencePositionCoach::DEFENSELINE_MARGIN)) {
                return std::nullopt;
            }
            Vector2 endPos = point + (lineToSideOne + lineToSideTwo) *
                                     0.5;// this defines the line on which the bisector lies.
            // now compute intersection with the defense area and return this (if it exists)
            return world::field->lineIntersectionWithDefenceArea(true, point, endPos,
                                                                 DefencePositionCoach::DEFENSELINE_MARGIN);
        }

/// gets the furthest position at which an obstacle will block the entire Angle
/// intuitively you can understand this as the closest point to which a circle of collissionRadius 'fits' in between the two lines
        Vector2 getBlockPoint(const Line &openGoalSegment, const Vector2 &point,
                              double collisionRadius) {
            //compute the bisector of the angle of point and the two ends of the openGoalSegment
            Vector2 lineToSideOne = (openGoalSegment.first - point).normalize();
            Vector2 lineToSideTwo = (openGoalSegment.second - point).normalize();
            Vector2 endPos = point + (lineToSideOne + lineToSideTwo)
                                     *
                                     0.5;// ending point on the bisector, which always just intersects the goalLine
            // compute the furthest distance at which the entire segment is blocked
            double theta = lineToSideOne.angle() - (endPos - point).angle(); // half of the angle of the bisector
            double collisionDist = collisionRadius / sin(theta);
            double angle = (endPos - point).angle();
            Vector2 FurthestBlock = point + Vector2(collisionDist, 0).rotate(angle);
            return FurthestBlock;
        }

// if the line hits the defence area shorten it, otherwise just return the original line
        Line shortenLineForDefenseArea(const Vector2 &lineStart, const Vector2 &lineEnd,
                                       double defenseMargin) {
            std::optional<Vector2> intersectPos = world::field->lineIntersectionWithDefenceArea(true, lineStart,
                                                                                                lineEnd,
                                                                                                defenseMargin);
            return std::make_pair(
                    lineStart, (intersectPos ? *intersectPos : lineEnd)
            );
        }

        void removeBotFromWorld(world::WorldData &world, int id, bool ourTeam) {
            auto &robots = ourTeam ? world.us : world.them;
            robots.erase(std::remove_if(robots.begin(), robots.end(),
                                        [id](const world::Robot::RobotPtr &robot) { return robot->id == id; }));
        }

        Vector2 getMostDangerousPos(const world::WorldData &world) {
            if (world.ball->getVel().length() > 0.5) {
                return world.ball->getPos() + world.ball->getVel() * 0.3;
            }
            return world.ball->getPos();
        }

        std::vector<std::pair<PossiblePass, double>> createPassesAndDanger(const world::WorldData &world) {
            std::vector<std::pair<PossiblePass, double>> passWithScore;
            // check the passes from the robot towards every other bot and calculate their danger
            for (const auto &theirBot : world.them) {
                //TODO: perhaps ignore robots we have already covered here. The score should be gutted regardless.
                PossiblePass pass(*theirBot, world.ball->getPos());
                // check how dangerous the pass is in our simulated world (pass.score(world))
                passWithScore.emplace_back(pass, pass.score(world));
            }
            return passWithScore;

        }

        std::vector<PossiblePass> sortPassesByDanger(
                std::vector<std::pair<PossiblePass, double>> &passWithScore) {
            //order passes from most dangerous to least
            std::sort(passWithScore.begin(), passWithScore.end(),
                      [](std::pair<PossiblePass, double> &left, std::pair<PossiblePass, double> &right) {
                          return left.second > right.second;
                      });
            std::vector<PossiblePass> passes;
            passes.reserve(passWithScore.size());
            for (const auto &passScore : passWithScore) {
                passes.push_back(passScore.first);
            }
            return passes;
        }

        std::vector<PossiblePass> createPassesSortedByDanger(const rtt::ai::world::WorldData &world) {
            auto passed = createPassesAndDanger(world);
            return sortPassesByDanger(passed);
        }


        DefenderBot createBlockToGoal(const PossiblePass &pass, double aggressionFactor, const Line &blockLine) {
            return {
                    0,
                    getPosOnLine(blockLine, aggressionFactor),
                    getOrientation(blockLine),
                    pass.toBot.id,
                    botType::BLOCKTOGOAL,
            };
        }

        DefenderBot createBlockToGoal(const PossiblePass &pass, const Vector2 &position, const Line &blockLine) {
            return {
                    0,
                    position,
                    getOrientation(blockLine),
                    pass.toBot.id,
                    botType::BLOCKTOGOAL
            };
        }

        DefenderBot createBlockOnLine(const PossiblePass &pass, const Vector2 &blockPos) {
            return {
                    0,
                    blockPos,
                    getOrientation({pass.endPos, blockPos}),
                    pass.toBot.id,
                    botType::BLOCKONLINE
            };
        }

        std::optional<Vector2> blockOnDefenseAreaLine(const PossiblePass &pass, const world::WorldData &world) {
            auto visibleParts = world::field->getVisiblePartsOfGoal(true, pass.endPos, world);
            // get the largest segment
            std::sort(visibleParts.begin(), visibleParts.end(),
                      [](const Line &a, const Line &b) {
                          return abs(a.second.y - a.first.y) > abs(b.second.y - b.first.y);
                      });
            if (!visibleParts.empty()) {
                return blockOnDefenseLine(visibleParts[0], pass.endPos);
            }
            return std::nullopt;
        }

/// checks for a given pass in a simulatedWorld if we can block it's receiver shot to goal and returns a line on which to stand if this is the case
        std::optional<Line> blockToGoalLine(const PossiblePass &pass, const world::WorldData &world) {
            // get the blockLine segment from the ending position of the pass
            auto visibleParts = world::field->getVisiblePartsOfGoal(true, pass.endPos, world);
            // get the largest segment (sort by size)
            std::sort(visibleParts.begin(), visibleParts.end(),
                      [](const Line &a, const Line &b) {
                          return abs(a.second.y - a.first.y) > abs(b.second.y - b.first.y);
                      });
            if (!visibleParts.empty()) {
                auto blockLine = getBlockLineSegment(visibleParts[0], pass.endPos);
                return blockLine;
            }
            return std::nullopt;
        }

/// searches the most dangerous position and then gets the segment which blocks that (if it exists/is possible)
        std::optional<Line> blockBallLine(const world::WorldData &world) {
            if (world.ball) {
                Vector2 mostDangerousPos = getMostDangerousPos(world);
                if (world::field->pointIsInField(mostDangerousPos, -0.1)) {
                    return getBlockLineSegment(world::field->getGoalSides(true), mostDangerousPos);
                }
            }
            return std::nullopt;
        }

        DefenderBot createBlockBall(const Line &blockLine) {
            // TODO: handle special handling for the case where we can't block using this bot (e.g. try to make keeper actively block?)
            return {
                    0,
                    findPositionForBlockBall(blockLine),
                    getOrientation(blockLine),
                    world::world->whichRobotHasBall(THEIR_ROBOTS) ? world::world->whichRobotHasBall(THEIR_ROBOTS)->id
                                                                  : -1,
                    botType::BLOCKBALL
            };
        }

        Vector2 findPositionForBlockBall(const Line &blockLine) {
            // if the blocking position is way away from our goal keep the robot on our side
            double maxForwardLineX = maxX();
            Vector2 position = getPosOnLine(blockLine, 0.1);
            if (position.x > maxForwardLineX) {
                double fieldWidth = world::field->get_field().field_width();
                Vector2 bottomLine(maxForwardLineX, -fieldWidth * 0.5);
                Vector2 topLine(maxForwardLineX, fieldWidth * 0.5);
                Vector2 intersect = control::ControlUtils::twoLineIntersection(blockLine.first, blockLine.second,
                                                                               bottomLine,
                                                                               topLine);
                return intersect;
            }
            return position;
        }

    }
} // rtt::ai::coach
