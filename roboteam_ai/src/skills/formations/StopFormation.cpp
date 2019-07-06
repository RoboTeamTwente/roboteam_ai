
#include "StopFormation.h"
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/interface/api/Input.h>
#include "../../control/Hungarian.h"

namespace rtt {
namespace ai {

std::shared_ptr<std::vector<std::shared_ptr<world::Robot>>> StopFormation::robotsInFormation = nullptr;

StopFormation::StopFormation(std::string name, bt::Blackboard::Ptr blackboard)
        : Formation(name, blackboard) {
    robotsInFormation = std::make_shared<std::vector<std::shared_ptr<world::Robot>>>();
}

// adapt to the change of robot amount in formation
void StopFormation::updateFormation() {
    targetLocation = getFormationPosition();
    robotsInFormationMemory = robotsInFormationPtr()->size();
}

Vector2 StopFormation::getFormationPosition() {

    //failsafe to prevent segfaults
    int amountOfRobots = robotsInFormation->size();
    if (amountOfRobots <= 0) {
        return {};
    }

    std::vector<int> robotIds;
    for (auto & i : *robotsInFormation) {
        if (robotIds.size() < 8) { // check for amount of robots, we dont want more than 8
            robotIds.push_back(i->id);
        }
    }

    rtt::HungarianAlgorithm hungarian;
    auto shortestDistances = hungarian.getRobotPositions(robotIds, true, getStopPositions().at(amountOfRobots-1));
    return shortestDistances.at(robot->id);
}

std::shared_ptr<std::vector<world::World::RobotPtr>> StopFormation::robotsInFormationPtr() {
    return robotsInFormation;
}

// determine the angle where the robot should point to (in position)
void StopFormation::setFinalAngle() {
    Vector2 targetToLookAtLocation = world::world->getBall()->pos;
    command.w = static_cast<float>((targetToLookAtLocation - robot->pos).angle());
}

std::vector<std::vector<Vector2>> StopFormation::getStopPositions() {
    auto field = world::field->get_field();

    auto pp = world::field->getPenaltyPoint(true); // penalty point

    auto defenseAreaLineA = world::field->get_field().left_penalty_line.begin;
    auto defenseAreaLineB = world::field->get_field().left_penalty_line.end;

    // divide the upper and bottom lines of the defense area and store those values.
    auto dTopY = fmax(defenseAreaLineA.y, defenseAreaLineB.y);
    auto dBtmY = fmin(defenseAreaLineA.y, defenseAreaLineB.y);
    auto defAreaHeight = fabs(dTopY - dBtmY);

    // the following statements specify useful stop positions between the ball and the goal
    auto ourGoalCenterToBall = ball->pos - world::field->get_our_goal_center();
    auto ballToOurGoalCenter = world::field->get_our_goal_center() - ball->pos;

    double distanceFromGoal = ball->pos.x > 0.0 ? 4.5 : 2.0;
    double distanceToBall = 1.0;

    if (ball->pos.x > 0.0) { // if the ball is on their side
        distanceFromGoal = 4.0;
    } else { // if the ball is on our side
        distanceFromGoal = (fabs(ball->pos.y) < 1.2) ? 1.6 : 2.2;
    }


    Vector2 closeToBallA = ball->pos + ballToOurGoalCenter.stretchToLength(distanceToBall).rotate(- sin(Constants::ROBOT_RADIUS()/distanceToBall));
    Vector2 closeToBallB = ball->pos + ballToOurGoalCenter.stretchToLength(distanceToBall).rotate(sin(Constants::ROBOT_RADIUS()/distanceToBall));

    // for one robot between ball and our goal
    Vector2 betweenGoalAndBallPosition = world::field->get_our_goal_center() + ourGoalCenterToBall.stretchToLength(distanceFromGoal);
    Vector2 betweenGoalAndBallPositionForwards = ourGoalCenterToBall.stretchToLength(distanceFromGoal).stretchToLength(distanceFromGoal+3*Constants::ROBOT_RADIUS());

    // for multiple robots between ball and our goal
    Vector2 diff = betweenGoalAndBallPosition + world::field->get_our_goal_center();
    Vector2 betweenGoalAndBallPositionA =  ourGoalCenterToBall.stretchToLength(distanceFromGoal).rotate(- sin(Constants::ROBOT_RADIUS()/distanceFromGoal)) + world::field->get_our_goal_center();
    Vector2 betweenGoalAndBallPositionB =  ourGoalCenterToBall.stretchToLength(distanceFromGoal).rotate(sin(Constants::ROBOT_RADIUS()/distanceFromGoal)) + world::field->get_our_goal_center();
    Vector2 betweenGoalAndBallPositionC =  ourGoalCenterToBall.stretchToLength(distanceFromGoal).rotate(2*sin(Constants::ROBOT_RADIUS()/distanceFromGoal)) + world::field->get_our_goal_center();
    Vector2 betweenGoalAndBallPositionD =  ourGoalCenterToBall.stretchToLength(distanceFromGoal).rotate(-2*sin(Constants::ROBOT_RADIUS()/distanceFromGoal)) + world::field->get_our_goal_center();

    Vector2 basicOffensivePositionA = {-1, 0.0};
    Vector2 basicOffensivePositionB = {-1, -(field.field_width*0.5-1.5)};
    Vector2 basicOffensivePositionC = {-1, (field.field_width*0.5-1.5)};

    double offset = 0.3;
    Vector2 inFrontOfDefenseAreaPositionA;
    Vector2 inFrontOfDefenseAreaPositionB;
    Vector2 inFrontOfDefenseAreaPositionC;
    double goal_width=world::field->get_field().goal_width;
    if (ball->pos.y>goal_width){
        inFrontOfDefenseAreaPositionA= {pp.x + offset, 0};
        inFrontOfDefenseAreaPositionB= {pp.x + offset, dBtmY};
        inFrontOfDefenseAreaPositionC = {pp.x + offset, dTopY};
    }
    else if (ball->pos.y<-goal_width){
        inFrontOfDefenseAreaPositionA= {pp.x + offset, 0};
        inFrontOfDefenseAreaPositionB= {pp.x + offset, dTopY};
        inFrontOfDefenseAreaPositionC = {pp.x + offset, dBtmY};
    }
    else {
        if (ball->pos.y>0){
            inFrontOfDefenseAreaPositionA= {pp.x + offset, dBtmY};
            inFrontOfDefenseAreaPositionB= {pp.x + offset, dTopY};
            inFrontOfDefenseAreaPositionC = {pp.x + offset, 0};
        }
        else{
            inFrontOfDefenseAreaPositionA= {pp.x + offset, dTopY};
            inFrontOfDefenseAreaPositionB= {pp.x + offset, dBtmY};
            inFrontOfDefenseAreaPositionC = {pp.x + offset, 0};
        }
    }

    std::vector<std::vector<Vector2>> targetLocations = {
            {betweenGoalAndBallPosition
            },

            {betweenGoalAndBallPositionA,
             betweenGoalAndBallPositionB
            },

            {betweenGoalAndBallPositionA,
             betweenGoalAndBallPositionB,
             inFrontOfDefenseAreaPositionA
            },

            {betweenGoalAndBallPositionA,
             betweenGoalAndBallPositionB,
             inFrontOfDefenseAreaPositionB,
             inFrontOfDefenseAreaPositionC
            },

            {betweenGoalAndBallPositionA,
             betweenGoalAndBallPositionB,
             inFrontOfDefenseAreaPositionB,
             inFrontOfDefenseAreaPositionC,
             inFrontOfDefenseAreaPositionA
            },

            {betweenGoalAndBallPositionA,
             betweenGoalAndBallPositionB,
             inFrontOfDefenseAreaPositionB,
             inFrontOfDefenseAreaPositionC,
             inFrontOfDefenseAreaPositionA,
             basicOffensivePositionA
            },

            {betweenGoalAndBallPositionA,
             betweenGoalAndBallPositionB,
             inFrontOfDefenseAreaPositionB,
             inFrontOfDefenseAreaPositionC,
             inFrontOfDefenseAreaPositionA,
             basicOffensivePositionB,
             basicOffensivePositionC
            },

            {betweenGoalAndBallPositionA,
             betweenGoalAndBallPositionB,
             betweenGoalAndBallPositionC,
             inFrontOfDefenseAreaPositionB,
             inFrontOfDefenseAreaPositionC,
             inFrontOfDefenseAreaPositionA,
             basicOffensivePositionB,
             basicOffensivePositionC
            }
    };
    return targetLocations;
}

}
}