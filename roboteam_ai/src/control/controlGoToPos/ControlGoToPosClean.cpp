//
// Created by rolf on 5-2-19.
//

#include "ControlGoToPosClean.h"
namespace rtt {
namespace ai {
namespace control {
///add a 'normal' point
void ControlGoToPosClean::PathPoint::addChild(
        std::shared_ptr<PathPoint> middleChild) {
    if (! middle) {
        middle = std::move(middleChild);
    }
    else {
        std::cout << "Child already claimed!!" << std::endl;
    }
}
///makes the two branches going to different direction
void ControlGoToPosClean::PathPoint::addChildren(
        std::shared_ptr<PathPoint> leftChild,
        std::shared_ptr<PathPoint> rightChild) {
    if (! left) {
        left = std::move(leftChild);
    }
    else {
        std::cout << "Left Child already claimed!!" << std::endl;
    }
    if (! right) {
        right = std::move(rightChild);
    }
    else {
        std::cout << "Right Child already claimed!!" << std::endl;
    }
}

///BackTrack until desired time or until Root
std::shared_ptr<ControlGoToPosClean::PathPoint> ControlGoToPosClean::PathPoint::backTrack(double backTime) {
    if (! parent) { return shared_from_this(); }
    if (backTime > t) { return shared_from_this(); }
    else return parent->backTrack(backTime);
}
void ControlGoToPosClean::drawCross(Vector2 &pos) {
// draws a cross for the display
    float dist = 0.004f;
    for (int i = - 7; i < 8; i ++) {
        for (int j = - 1; j < 2; j += 2) {
            Vector2 data = pos + (Vector2) {dist*i, dist*j*i};
            displayData.push_back(data);
        }
    }
}
/// Turn ball avoidance off and on
void ControlGoToPosClean::setAvoidBall(bool _avoidBall) {
    avoidBall = _avoidBall;
}
/// Make the Robot able to go outside of the field
void ControlGoToPosClean::setCanGoOutsideField(bool _canGoOutsideField) {
    canGoOutsideField = _canGoOutsideField;
}
void ControlGoToPosClean::initializePID() {
    velPID.reset();
    velPID.setPID(constants::standard_luth_P,
            constants::standard_luth_P,
            constants::standard_luth_P);

    posPID.reset();
    posPID.setPID(constants::standard_luth_P,
            constants::standard_luth_P,
            constants::standard_luth_P);
}
void ControlGoToPosClean::checkInterfacePID() {
    if (velPID.getP() != interface::InterfaceValues::getLuthP() ||
            velPID.getI() != interface::InterfaceValues::getLuthI() ||
            velPID.getD() != interface::InterfaceValues::getLuthD()) {
        velPID.reset();
        velPID.setPID(interface::InterfaceValues::getLuthP(),
                interface::InterfaceValues::getLuthI(),
                interface::InterfaceValues::getLuthD());

        posPID.reset();
        posPID.setPID(interface::InterfaceValues::getLuthP(),
                interface::InterfaceValues::getLuthI(),
                interface::InterfaceValues::getLuthD());
    }
}
void ControlGoToPosClean::drawInInterface() {

    std::vector<std::pair<rtt::Vector2, QColor>> displayColorData = {{{}, {}}};
    for (auto &displayAll : displayData)
        displayColorData.emplace_back(displayAll, Qt::green);
}
Vector2 ControlGoToPosClean::goToPos(std::shared_ptr<roboteam_msgs::WorldRobot> robot, Vector2 targetPos) {
    initializePID();
    checkInterfacePID();
}
ControlGoToPosClean::PathPoint ControlGoToPosClean::computeNewPoint(PathPoint oldPoint,
        Vector2 subTarget) {
    PathPoint newPoint;
    newPoint.parent=oldPoint;
    newPoint.t=oldPoint.t+dt;
    newPoint.currentTarget=subTarget;
    //ODE model:
    Vector2 targetVel=(oldPoint.currentTarget-oldPoint.pos).normalize()*maxVel;
    newPoint.acc=(targetVel-oldPoint.vel).normalize()*maxAcc;
    newPoint.vel=oldPoint.vel+newPoint.acc*dt;
    newPoint.pos=oldPoint.pos+newPoint.vel*dt;
    displayData.push_back(newPoint.pos);
    return newPoint;
}
//
bool ControlGoToPosClean::checkCollission(PathPoint point) {
    roboteam_msgs::World world=World::futureWorld(point.t);
    for (auto bot: world.us){
        if(bot.id!=robotID){
            if ((Vector2(bot.pos)-point.pos).length()<(2*constants::ROBOT_RADIUS_MAX)){
                return true;
            }
        }
    }
    for (auto bot: world.them){
        if ((Vector2(bot.pos)-point.pos).length()<(2*constants::ROBOT_RADIUS_MAX)){
            return true;
        }
    }
    if (avoidBall){
       if ((Vector2(world.ball.pos)-point.pos).length()<(2*constants::ROBOT_RADIUS_MAX)){
           return true;
       }
    }
    return false;
}
}// control
}// ai
}// rtt