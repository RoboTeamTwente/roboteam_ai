//
// Created by rolf on 12/12/18.
//

#include "DemoInterceptBall.h"
#include "roboteam_ai/src/interface/api/Input.h"
#include "roboteam_ai/src/world/Field.h"

namespace rtt {
namespace ai {

DemoInterceptBall::DemoInterceptBall(rtt::string name, bt::Blackboard::Ptr blackboard)
        :InterceptBall(std::move(name), std::move(blackboard)) { };


Vector2 DemoInterceptBall::computeInterceptPoint(Vector2 startBall, Vector2 endBall) {
    Vector2 interceptionPoint;
    if (keeper) {
        // Depends on two keeper Constants in Constants!
        Arc keeperCircle = createKeeperArc();
        std::pair<boost::optional<Vector2>, boost::optional<Vector2>> intersections = keeperCircle.intersectionWithLine(
                startBall, endBall);
        if (intersections.first && intersections.second) {
            double dist1 = (Vector2(robot->pos) - *intersections.first).length();
            double dist2 = (Vector2(robot->pos) - *intersections.second).length();
            if (dist2 < dist1) {
                interceptionPoint = *intersections.second;
            }
            else {
                interceptionPoint = *intersections.first;
            }
        }
        else if (intersections.first) {
            interceptionPoint = *intersections.first;
        }
        else if (intersections.second) {
            interceptionPoint = *intersections.second;
        }
        else {
            // if the Line does not intercept it usually means the ball is coming from one of the corners-ish to the keeper
            // For now we pick the closest point to the (predicted) line of the ball
            Line shotLine(startBall, endBall);
            interceptionPoint = shotLine.project(robot->pos);
        }
    }
    else {
        // For now we pick the closest point to the (predicted) line of the ball for any 'regular' interception
        Line shotLine(startBall, endBall);
        interceptionPoint = shotLine.project(robot->pos);
    }
    return interceptionPoint;
}

Arc DemoInterceptBall::createKeeperArc() {
    double goalwidth = rtt::ai::world::field->get_field().goal_width;
    Vector2 goalPos = rtt::ai::world::field->get_their_goal_center();
    double diff = rtt::ai::Constants::KEEPER_POST_MARGIN() - rtt::ai::Constants::KEEPER_CENTREGOAL_MARGIN();

    double radius = diff*0.5 + goalwidth*goalwidth/(8*diff); //Pythagoras' theorem.
    double angle = asin(goalwidth/2/radius); // maximum angle (at which we hit the posts)
    Vector2 center = Vector2(goalPos.x - rtt::ai::Constants::KEEPER_CENTREGOAL_MARGIN() - radius, 0);
    return diff > 0 ? rtt::Arc(center, radius, M_PI - angle, angle - M_PI) :
           rtt::Arc(center, radius, angle, - angle);
}

}//ai
}//rtt