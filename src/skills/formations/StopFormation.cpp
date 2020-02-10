#include "skills/formations/StopFormation.h"
#include <interface/api/Input.h>
#include <world/FieldComputations.h>

namespace rtt::ai {

std::shared_ptr<std::vector<std::shared_ptr<world::Robot>>> StopFormation::robotsInFormation = nullptr;

StopFormation::StopFormation(std::string name, bt::Blackboard::Ptr blackboard) : Formation(name, blackboard) {
    robotsInFormation = std::make_shared<std::vector<std::shared_ptr<world::Robot>>>();
}

// adapt to the change of robot amount in formation
void StopFormation::updateFormation() {
    targetLocation = getFormationPosition();
    robotsInFormationMemory = robotsInFormationPtr()->size();
}

Vector2 StopFormation::getFormationPosition() {
    // failsafe to prevent segfaults
    int amountOfRobots = robotsInFormation->size();
    if (amountOfRobots <= 0) {
        return {};
    }

    auto formationPositions = getStopPositions().at(amountOfRobots - 1);
    std::vector<Vector2> properPositions;
    for (auto pos : formationPositions) {
        if (!positionShouldBeAvoided(pos)) {
            properPositions.push_back(pos);
        }
    }

    int amountToTake = amountOfRobots - properPositions.size();
    std::vector<Vector2> newProposals = getProperPositions(amountToTake);

    for (Vector2 proposal : newProposals) {
        properPositions.push_back(proposal);
    }

    return getOptimalPosition(robot->id, *robotsInFormation, properPositions);
}

std::shared_ptr<std::vector<world::World::RobotPtr>> StopFormation::robotsInFormationPtr() { return robotsInFormation; }

// determine the angle where the robot should point to (in position)
void StopFormation::setFinalAngle() {
    Vector2 targetToLookAtLocation = world->getBall()->getPos();
    command.set_w((targetToLookAtLocation - robot->pos).angle());
}

std::vector<std::vector<Vector2>> StopFormation::getStopPositions() {
    auto pp = FieldComputations::getPenaltyPoint(*field, true);  // penalty point
    auto dTopY = field->getPenaltyTopY();
    auto dBtmY = field->getPenaltyBottomY();

    // the following statements specify useful stop positions between the ball and the goal
    auto ourGoalCenterToBall = ball->getPos() - field->getOurGoalCenter();
    auto ballToOurGoalCenter = field->getOurGoalCenter() - ball->getPos();

    double distanceFromGoal;
    double distanceToBall = 1.0;

    if (ball->getPos().x > 0.0) {  // if the ball is on their side
        distanceFromGoal = 4.0;
    } else {  // if the ball is on our side
        distanceFromGoal = (fabs(ball->getPos().y) < 1.2) ? 1.6 : 2.2;
    }

    Vector2 closeToBallMiddle = ball->getPos() + ballToOurGoalCenter.stretchToLength(distanceToBall);

    Vector2 closeToBallA = ball->getPos() + ballToOurGoalCenter.stretchToLength(distanceToBall).rotate(-sin(Constants::ROBOT_RADIUS() / distanceToBall));
    Vector2 closeToBallB = ball->getPos() + ballToOurGoalCenter.stretchToLength(distanceToBall).rotate(sin(Constants::ROBOT_RADIUS() / distanceToBall));

    // for one robot between ball and our goal
    Vector2 betweenGoalAndBallPosition = field->getOurGoalCenter() + ourGoalCenterToBall.stretchToLength(distanceFromGoal);
    Vector2 betweenGoalAndBallPositionForwards = ourGoalCenterToBall.stretchToLength(distanceFromGoal).stretchToLength(distanceFromGoal + 3 * Constants::ROBOT_RADIUS());

    // for multiple robots between ball and our goal
    Vector2 diff = betweenGoalAndBallPosition + field->getOurGoalCenter();
    Vector2 betweenGoalAndBallPositionA =
        ourGoalCenterToBall.stretchToLength(distanceFromGoal).rotate(-sin(Constants::ROBOT_RADIUS() / distanceFromGoal)) + field->getOurGoalCenter();
    Vector2 betweenGoalAndBallPositionB =
        ourGoalCenterToBall.stretchToLength(distanceFromGoal).rotate(sin(Constants::ROBOT_RADIUS() / distanceFromGoal)) + field->getOurGoalCenter();
    Vector2 betweenGoalAndBallPositionC =
        ourGoalCenterToBall.stretchToLength(distanceFromGoal).rotate(2 * sin(Constants::ROBOT_RADIUS() / distanceFromGoal)) + field->getOurGoalCenter();
    Vector2 betweenGoalAndBallPositionD =
        ourGoalCenterToBall.stretchToLength(distanceFromGoal).rotate(-2 * sin(Constants::ROBOT_RADIUS() / distanceFromGoal)) + field->getOurGoalCenter();

    Vector2 basicOffensivePositionA = {-1, 0.0};

    double offset = 0.3;
    Vector2 inFrontOfDefenseAreaPositionA;
    Vector2 inFrontOfDefenseAreaPositionB;
    Vector2 inFrontOfDefenseAreaPositionC;
    double goal_width = field->getGoalWidth();
    if (ball->getPos().y > goal_width) {
        inFrontOfDefenseAreaPositionA = {pp.x + offset, 0};
        inFrontOfDefenseAreaPositionB = {pp.x + offset, dBtmY};
        inFrontOfDefenseAreaPositionC = {pp.x + offset, dTopY};
    } else if (ball->getPos().y < -goal_width) {
        inFrontOfDefenseAreaPositionA = {pp.x + offset, 0};
        inFrontOfDefenseAreaPositionB = {pp.x + offset, dTopY};
        inFrontOfDefenseAreaPositionC = {pp.x + offset, dBtmY};
    } else {
        if (ball->getPos().y > 0) {
            inFrontOfDefenseAreaPositionA = {pp.x + offset, dBtmY};
            inFrontOfDefenseAreaPositionB = {pp.x + offset, dTopY};
            inFrontOfDefenseAreaPositionC = {pp.x + offset, 0};
        } else {
            inFrontOfDefenseAreaPositionA = {pp.x + offset, dTopY};
            inFrontOfDefenseAreaPositionB = {pp.x + offset, dBtmY};
            inFrontOfDefenseAreaPositionC = {pp.x + offset, 0};
        }
    }

    std::vector<std::vector<Vector2>> targetLocations = {
        {closeToBallMiddle},

        {closeToBallA, closeToBallB},

        {betweenGoalAndBallPositionA, betweenGoalAndBallPositionB, closeToBallMiddle},

        {closeToBallA, closeToBallB, betweenGoalAndBallPositionA, betweenGoalAndBallPositionB},

        {closeToBallA, closeToBallB, betweenGoalAndBallPositionA, betweenGoalAndBallPositionB, inFrontOfDefenseAreaPositionA},

        {closeToBallA, closeToBallB, betweenGoalAndBallPositionA, betweenGoalAndBallPositionB, inFrontOfDefenseAreaPositionA, basicOffensivePositionA},

        {closeToBallA, closeToBallB, betweenGoalAndBallPositionA, betweenGoalAndBallPositionB, inFrontOfDefenseAreaPositionB, inFrontOfDefenseAreaPositionC,
         basicOffensivePositionA},

        {closeToBallA, closeToBallB, betweenGoalAndBallPositionA, betweenGoalAndBallPositionB, betweenGoalAndBallPositionC, inFrontOfDefenseAreaPositionB,
         inFrontOfDefenseAreaPositionC, inFrontOfDefenseAreaPositionA}};
    return targetLocations;
}

bool StopFormation::positionShouldBeAvoided(Vector2 pos) {
    return (pos.dist(ball->getPos()) < 0.9 || !FieldComputations::pointIsInField(*field, pos));
}

std::vector<Vector2> StopFormation::getProperPositions(int amount) {
    std::vector<Vector2> properPositions;
    std::vector<Vector2> proposals;

    // near the corners
    proposals.push_back({field->getLeftmostX() + 1.0, field->getBottommostY() + 1.5});
    proposals.push_back({field->getLeftmostX() + 1.0, field->getTopmostY() - 1.5});

    // somewhere in the middle of our half
    proposals.push_back({-field->getFieldLength() * 0.3, field->getBottommostY() + 1.5});
    proposals.push_back({-field->getFieldLength() * 0.3, field->getTopmostY() - 1.5});

    // offensive
    proposals.push_back({-1, field->getBottommostY() + 1.5});
    proposals.push_back({-1, field->getTopmostY() - 1.5});
    proposals.push_back({-1, 0});

    for (auto proposal : proposals) {
        if (!positionShouldBeAvoided(proposal) && amount > 0) {
            properPositions.push_back(proposal);
            amount--;
        }
    }

    while (amount > 0) {
        properPositions.push_back(field->getOurGoalCenter());
        amount--;
    }

    return properPositions;
}

}  // namespace rtt::ai