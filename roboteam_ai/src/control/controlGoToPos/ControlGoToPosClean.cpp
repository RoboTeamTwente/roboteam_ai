//
// Created by rolf on 5-2-19.
//

#include "ControlGoToPosClean.h"
#include <chrono>
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
void ControlGoToPosClean::PathPoint::addBranch(std::shared_ptr<PathPoint> branchChild){
    branches.push_back(branchChild);
    return;
}

///BackTrack until desired time or until Root
std::shared_ptr<ControlGoToPosClean::PathPoint> ControlGoToPosClean::PathPoint::backTrack(double backTime) {
    if (! parent) { return shared_from_this();}
    if (backTime > t) { return shared_from_this(); }
    else return parent->backTrack(backTime);
}

///backTracks the path from endPoint until it hits root and outputs in order from root->endPoint
std::vector<ControlGoToPosClean::PathPoint> ControlGoToPosClean::backTrackPath(std::shared_ptr<PathPoint> endPoint, std::shared_ptr<PathPoint> root){
    std::vector<PathPoint> path;
    std::shared_ptr<PathPoint> point=endPoint;
    while(point){
        path.push_back(*point);
        if (point==root){
            break;
        }
        point=point->parent;
    }
    std::reverse(path.begin(),path.end()); // everything is from back to forth
    ros::Time end = ros::Time::now();
    return path;
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
    for (auto &displayAll : displayData){
        displayColorData.emplace_back(displayAll, Qt::green);
    }
    for (auto &displayPath : path){
        displayColorData.emplace_back(displayPath.pos,Qt::red);
    }
    rtt::ai::interface::Drawer::setGoToPosLuThPoints(robotID, displayColorData);
}
Vector2 ControlGoToPosClean::goToPos(std::shared_ptr<roboteam_msgs::WorldRobot> robot, Vector2 targetPos) {
    initializePID();
    checkInterfacePID();
    bool recalculate = false;
    bool nicePath= true;
    recalculate=true;
//    // if the targetPos or robot Id changed we have to recalculate.
//    if (finalTargetPos!=targetPos||robotID!=robot->id){
//        recalculate=true;
//    }
////    else{
////        recalculate=robotOnPath();
////    }
    if (recalculate){
        nicePath=false;
        finalTargetPos=targetPos;
        displayData={};
        velPID.reset();
        posPID.reset();
        path.clear();
        //pathQueue=std::priority_queue<std::shared_ptr<PathPoint>,std::vector<std::shared_ptr<PathPoint>>,CustomCompare>();
        //create root node and start searching
        std::shared_ptr<PathPoint> root=std::make_shared<PathPoint>();
        root->currentTarget=targetPos;
        root->pos=robot->pos;
        root->vel=robot->vel;
        root->acc={0,0}; //Assumed for now but could be known from world state/previous commands
        root->t=0;
        root->collisions=0;
        robotID=robot->id;
        //draw final destination
        drawCross(targetPos);
        // find a path
        std::chrono::high_resolution_clock::time_point start=std::chrono::high_resolution_clock::now();
        time=std::chrono::milliseconds(0);
        path=tracePath(root);
        std::chrono::high_resolution_clock::time_point end=std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> TotalTime=std::chrono::duration_cast<std::chrono::duration<double>> (end-start);
        std::cout<<"Other time"<<(TotalTime.count()-time.count())*1000<<" Time spent in function:"<<time.count()*1000 <<std::endl;
        //clear out the queue again so that on next ticks it's fine. It might still be problematic with memory leaks? Not sure
//        while(pathQueue.size()!=0){
//            pathQueue.pop();
//        }
        if (!path.empty()){ nicePath=true;}
    }
    drawInInterface();
    Vector2 command=computeCommand();
    return command;
}


std::vector<ControlGoToPosClean::PathPoint> ControlGoToPosClean::tracePath(std::shared_ptr<PathPoint> root) {
    std::vector<PathPoint> path={};
    auto compAS= [this](std::shared_ptr<PathPoint> lhs,std::shared_ptr<PathPoint> rhs){
      if (fmod(lhs->collisions,3)>fmod(rhs->collisions,3)){return true;}
      if (fmod(rhs->collisions,3)>fmod(lhs->collisions,3)){return false;}
      return (lhs->t+abs((lhs->currentTarget - finalTargetPos).length())/1.56)
              > (rhs->t+abs((rhs->currentTarget - finalTargetPos).length())/1.56);
    };
    auto compClose= [this](std::shared_ptr<PathPoint> lhs,std::shared_ptr<PathPoint> rhs){
        if (lhs->collisions>rhs->collisions){return true;}
        if (rhs->collisions>lhs->collisions){return false;}
      return (abs((lhs->currentTarget - finalTargetPos).length()+(lhs->pos-lhs->currentTarget).length()))
              > (abs((rhs->currentTarget - finalTargetPos).length()+(rhs->pos-rhs->currentTarget).length()));
    };
    std::priority_queue<std::shared_ptr<PathPoint>,std::vector<std::shared_ptr<PathPoint>>,decltype(compClose)> pathQueue( compClose);
    //create root of tree:
    pathQueue.push(root);
    ros::Time start=ros::Time::now();
    // start searching
    while(!pathQueue.empty()){

        std::shared_ptr<PathPoint> BranchStart=pathQueue.top();
        std::shared_ptr<PathPoint> point=BranchStart;
        while(true){
            std::shared_ptr<PathPoint> newPoint=computeNewPoint(point,point->currentTarget);
            point->addChild(newPoint);
            point=newPoint;

            // if we reach endpoint, return the path we found
            if (point->isCollission(finalTargetPos,0.1)){
                ros::Time end=ros::Time::now();
                //std::cout<<"HalfWay->END"<<(end-start).toSec()*1000<<std::endl;
                path=backTrackPath(point,root);
                return path;
            }
            // if we reach a halfway point, update the target to the final target again and push this new point to queue
            else if(point->isCollission(point->currentTarget,0.1)){
                ros::Time end=ros::Time::now();
                //std::cout<<"to halfway (reached):"<<(end-start).toSec()*1000<<std::endl;
                point->currentTarget=finalTargetPos;
                pathQueue.pop();
                pathQueue.push(point);
                if (point->middle){
                    std::cout<<"STOP RIGHT THERE!"<<std::endl;
                }
                break; // break from current while loop, we start looking at different branches again
            }
            // if we have collided with an obstacle; backtrack and branch to both sides using new intermediary points
            if (checkCollission(point)) {
                std::shared_ptr<PathPoint> newBranchStart = point->backTrack(point->t - 0.4);
                //std::shared_ptr<PathPoint> newBranchStart=BranchStart;
                std::vector<Vector2> targets = getNewTargets(point, newBranchStart->pos);
                pathQueue.pop();
                // both left and right targets for now
                for (auto newTarget :targets){
                    // Check if we are not creating a duplicate
                    if (!branchHasTarget(newBranchStart,newTarget)) {
                        // compute new point and add it to branch
                        std::shared_ptr<PathPoint> branch = computeNewPoint(newBranchStart, newTarget);
                        branch->collisions ++;
                        newBranchStart->addBranch(branch);
                        pathQueue.push(branch);
                    }
                }

                ros::Time end=ros::Time::now();
                //std::cout<<"to collision:"<<(end-start).toSec()*1000<<std::endl;
                break; // break from current while loop, we start looking at different branches again
            }
        }
        ros::Time now = ros::Time::now();
        if ((now - start).toSec()*1000 > 25*constants::MAX_CALCULATION_TIME) {
            std::cout << "Tick took too long!" << std::endl;
            //TODO we return an empty path now, we could also return the 'best' path that is closest to the robot perhaps?
            return {};
        }
    }
    return {};
}
bool ControlGoToPosClean::branchHasTarget(std::shared_ptr<PathPoint> newBranchStart, Vector2 target){
    for (auto child: newBranchStart->branches){
        if (child->currentTarget==target){
            return true;
        }
    }
    return false;
}
std::shared_ptr<ControlGoToPosClean::PathPoint> ControlGoToPosClean::computeNewPoint(std::shared_ptr<PathPoint> oldPoint,
        Vector2 subTarget) {
    std::shared_ptr<PathPoint> newPoint=std::make_shared<PathPoint>();
    newPoint->parent=oldPoint;
    newPoint->t=oldPoint->t+dt;
    newPoint->currentTarget=subTarget;
    //ODE model:
    Vector2 targetVel=(oldPoint->currentTarget-oldPoint->pos).normalize()*maxVel;
    newPoint->acc=(targetVel-oldPoint->vel).normalize()*maxAcc;
    newPoint->vel=oldPoint->vel+newPoint->acc*dt;
    newPoint->pos=oldPoint->pos+newPoint->vel*dt;
    newPoint->collisions=oldPoint->collisions;
    displayData.push_back(newPoint->pos);

    return newPoint;
}
bool ControlGoToPosClean::checkCollission(std::shared_ptr<PathPoint> point) {
    std::chrono::high_resolution_clock::time_point start=std::chrono::high_resolution_clock::now();
    roboteam_msgs::World world=World::futureWorld(point->t);
    for (auto bot: world.us){
        if(bot.id!=robotID){
            if (point->isCollission(bot.pos,2*constants::ROBOT_RADIUS_MAX)){
                std::chrono::high_resolution_clock::time_point end=std::chrono::high_resolution_clock::now();
                time=time+(end-start);
                return true;
            }
        }
    }
    for (auto bot: world.them){
        if (point->isCollission(bot.pos,2*constants::ROBOT_RADIUS_MAX)){
            std::chrono::high_resolution_clock::time_point end=std::chrono::high_resolution_clock::now();
            time=time+(end-start);
            return true;
        }
    }
    if (avoidBall){
       if (point->isCollission(world.ball.pos,constants::ROBOT_RADIUS_MAX+constants::BALL_RADIUS)){
           std::chrono::high_resolution_clock::time_point end=std::chrono::high_resolution_clock::now();
           time=time+(end-start);
           return true;
       }
    }
    std::chrono::high_resolution_clock::time_point end=std::chrono::high_resolution_clock::now();
    time=time+(end-start);
    return false;
}

Vector2 ControlGoToPosClean::findCollisionPos(std::shared_ptr<PathPoint> point){
    roboteam_msgs::World world=World::futureWorld(point->t);
    for (auto bot: world.us){
        if(bot.id!=robotID){
            if (point->isCollission(bot.pos,2*constants::ROBOT_RADIUS_MAX)){
                return bot.pos;
            }
        }
    }
    for (auto bot: world.them){
        if (point->isCollission(bot.pos,2*constants::ROBOT_RADIUS_MAX)){
            return bot.pos;
        }
    }
    if (avoidBall){
        if (point->isCollission(world.ball.pos,constants::ROBOT_RADIUS_MAX+constants::BALL_RADIUS)){
            return world.ball.pos;
        }
    }
    return {-42,42};

}
bool ControlGoToPosClean::PathPoint::isCollission(Vector2 target,double distance) {
    return (target-pos).length()<distance;
}
std::vector<Vector2> ControlGoToPosClean::getNewTargets(std::shared_ptr<PathPoint> collisionPoint, Vector2 startPos ) {
    // we find the centre of the object we are colliding with
    Vector2 collisionObjPos=findCollisionPos(collisionPoint);
    // then pick 2 targets left and right of it Preferably at 0.3 m, but closer if they are close to the endPoint.
    //double angle=(collisionObjPos-startPos).angle();
    double angle=collisionPoint->vel.angle();
    double avoidDistanceDefault=0.3;
    double objectDist=(collisionObjPos-finalTargetPos).length();
    if (objectDist<0.18) {objectDist=0.18;}
    double dist=fmin(objectDist,avoidDistanceDefault);
    Vector2 avoidDist(dist,0);
    Vector2 leftTarget=collisionObjPos+avoidDist.rotate(angle-M_PI_2);
    Vector2 rightTarget=collisionObjPos+avoidDist.rotate(angle+M_PI_2);
    std::vector<Vector2> newTargets={leftTarget,rightTarget};
    drawCross(leftTarget);
    drawCross(rightTarget);
    return newTargets;
}
Vector2 ControlGoToPosClean::computeCommand(){
    return{0,0};
}
}// control
}// ai
}// rtt