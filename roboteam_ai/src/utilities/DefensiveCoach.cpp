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

DefensiveCoach::PossiblePass::PossiblePass(roboteam_msgs::WorldRobot _toBot,
        roboteam_msgs::WorldRobot _fromBot,Vector2 ballPos)
        :
        toBot(_toBot),
        fromBot(_fromBot),
        startPos(ballPos),
        endPos(util::computeSimpleReceivePos(ballPos,_toBot.pos)){ };

double DefensiveCoach::PossiblePass::distance() {
    return (endPos - startPos).length();
}
bool DefensiveCoach::PossiblePass::obstacleObstructsPath(Vector2 obstPos, double obstRadius) {
    return util::distanceToLineWithEnds(obstPos,startPos,endPos)<=obstRadius;
}
int DefensiveCoach::PossiblePass::amountOfBlockers() {
    int total=0;
    for (auto bot : World::get_world().us){
        if (obstacleObstructsPath(bot.pos)){
            total++;
        }
    }
    return total;
}
///returns all passes that are not obstructed
std::vector<DefensiveCoach::PossiblePass> DefensiveCoach::getPossiblePassesThem() {
    std::vector<PossiblePass> possiblePasses;
    //first find if one of their bots has the ball and if so which bot
    int theirBotWithBall = World::whichBotHasBall(false);
    std::shared_ptr<roboteam_msgs::WorldRobot> passBot=World::getRobotForId(theirBotWithBall,false);
    if (theirBotWithBall == - 1||!passBot) {
        return possiblePasses;
    }
    for (auto bot: World::get_world().them) {
        if (bot.id != theirBotWithBall) {
            PossiblePass pass(bot,*passBot,World::getBall()->pos);
            int blockAmount=pass.amountOfBlockers();
            if (blockAmount==0){
                possiblePasses.push_back(pass);
            }
        }
    }
    return possiblePasses;
}
std::shared_ptr<std::pair<Vector2,Vector2>> DefensiveCoach::getBlockLineSegment(std::pair<Vector2,Vector2> openGoalSegment, Vector2 point,double collisionRadius){
    double margin=collisionRadius;
    //compute the bisector
    Vector2 lineToSideOne=openGoalSegment.first-point;
    Vector2 lineToSideTwo=(openGoalSegment.second-point).stretchToLength(lineToSideOne.length());
    Vector2 startPos=point+(lineToSideOne+lineToSideTwo).stretchToLength(collisionRadius);
    if (Field::pointIsInDefenceArea(startPos,true,margin)){
        return nullptr;
    }
    Vector2 endPos=point+(lineToSideOne+lineToSideTwo)*0.5;// this defines the line on which the bisector lies.
    double theta=lineToSideOne.angle()-(endPos-point).angle();
    double collisionDist=collisionRadius/sin(theta);
    Vector2 FurthestBlock=point+Vector2(collisionDist,0).rotate((endPos-point).angle());
    //check intersections with defense area
    auto field=Field::get_field();
    Vector2 goalLinePos1(- field.field_length*0.5, field.left_penalty_line.begin.y-margin);
    Vector2 goalLinePos2(- field.field_length*0.5, field.left_penalty_line.end.y+margin);
    Vector2 cornerPos1=Vector2(margin,-margin)+field.left_penalty_line.begin;
    Vector2 cornerPos2=Vector2(margin,margin)+field.left_penalty_line.end;
    if (field.left_penalty_line.begin.y>field.left_penalty_line.end.y) {
        goalLinePos1=Vector2(- field.field_length*0.5, field.left_penalty_line.begin.y+margin);
        goalLinePos2=Vector2(- field.field_length*0.5, field.left_penalty_line.end.y-margin);
        cornerPos1=Vector2(margin,margin)+field.left_penalty_line.begin;
        cornerPos2=Vector2(margin,-margin)+field.left_penalty_line.end;
    }
    // if it intersects with defense area return that,
    Vector2 intersectPos;
    if (util::lineSegmentsIntersect(point,FurthestBlock,goalLinePos1,cornerPos1)){
        intersectPos=util::twoLineIntersection(point,FurthestBlock,goalLinePos1,cornerPos1);
    }
    else if(util::lineSegmentsIntersect(point,FurthestBlock,cornerPos1,cornerPos2)){
        intersectPos=util::twoLineIntersection(point,FurthestBlock,cornerPos1,cornerPos2);
    }
    else if (util::lineSegmentsIntersect(point,FurthestBlock,cornerPos2,goalLinePos2)){
        intersectPos=util::twoLineIntersection(point,FurthestBlock,cornerPos2,goalLinePos2);
    }
    else{
        // return the original line
        std::pair<Vector2,Vector2> line=std::make_pair(startPos,FurthestBlock);
        std::shared_ptr<std::pair<Vector2,Vector2>> segment=std::make_shared<std::pair<Vector2,Vector2>>(line);
        return segment;
    }
    std::pair<Vector2,Vector2> line=std::make_pair(startPos,intersectPos);
    std::shared_ptr<std::pair<Vector2,Vector2>> segment=std::make_shared<std::pair<Vector2,Vector2>>(line);
    return segment;

}
std::shared_ptr<std::pair<Vector2,Vector2>> DefensiveCoach::blockBall(){
    if (World::getBall()) {
        auto goalSides = Field::getGoalSides(true);
        return getBlockLineSegment(goalSides,World::getBall()->pos,Constants::ROBOT_RADIUS()+Constants::BALL_RADIUS());
    }
    return nullptr;
}

std::vector<std::pair<Vector2,Vector2>> DefensiveCoach::getWholeBlockSegments(std::vector<Vector2> points){
    std::vector<std::pair<Vector2,Vector2>> segments;
    std::pair<Vector2,Vector2> goalsides=Field::getGoalSides(true);
    for (Vector2 point : points){
        auto segment=getBlockLineSegment(goalsides,point,Constants::ROBOT_RADIUS()+Constants::BALL_RADIUS());
        if (segment){
            segments.push_back(*segment);
        }
    }
    return segments;
}

}
}
}