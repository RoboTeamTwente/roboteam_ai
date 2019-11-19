//
// Created by kjhertenberg on 13-5-19.
//

#include "roboteam_proto/messages_robocup_ssl_detection.pb.h"
#include <kalman/WorldFilter.h>

namespace world {

    WorldFilter::WorldFilter() {
        blueBots.clear();
        yellowBots.clear();
        balls.clear();
    }

// if we get a new frame we update our observations
    void WorldFilter::addFrame(const proto::SSL_DetectionFrame &msg) {
        const double filterGrabDistance=0.5;
        std::lock_guard<std::mutex> lock(filterMutex);

        double timeCapture = msg.t_capture();
        uint cameraID = msg.camera_id();
        for (const proto::SSL_DetectionRobot &robot : msg.robots_yellow()) {
            bool addedBot=false;
            for (const auto &filter : yellowBots[robot.robot_id()]) {
                if (filter->distanceTo(robot.x(),robot.y())<filterGrabDistance){
                    filter->addObservation(robot,timeCapture);
                    addedBot=true;
                }
            }
            if (!addedBot){
                // We create a new filter if no filter close to the robot exists
                yellowBots[robot.robot_id()].push_back(std::make_unique<RobotFilter>(robot,timeCapture));
            }
        }
        for (const proto::SSL_DetectionRobot &robot : msg.robots_blue()) {
            bool addedBot=false;
            for (const auto &filter : blueBots[robot.robot_id()]) {
                if (filter->distanceTo(robot.x(),robot.y())<filterGrabDistance){
                    filter->addObservation(robot,timeCapture);
                    addedBot=true;
                }
            }
            if (!addedBot){
                // We create a new filter
                blueBots[robot.robot_id()].push_back(std::make_unique<RobotFilter>(robot,timeCapture));
            }
        }
        for (const proto::SSL_DetectionBall &detBall : msg.balls()) {
            bool addedBall=false;
            for (const auto &filter : balls) {
                if (filter->distanceTo(detBall.x(),detBall.y())<filterGrabDistance){
                    filter->addObservation(detBall,timeCapture);
                    addedBall=true;
                }
            }
            if(!addedBall){
                balls.push_back(std::make_unique<BallFilter>(detBall,timeCapture));
            }
        }

    }

//Creates a world message with the currently observed objects in it
    proto::World WorldFilter::getWorld(double time) {
        //First we update to the time we want packets at. Very important!
        update(time,true);
        proto::World world;
        world.set_time(time);
        for (const auto &kalmanYellowBotsOneId : yellowBots) {
            if (!kalmanYellowBotsOneId.second.empty()) {
                world.mutable_yellow()->Add(bestFilter(kalmanYellowBotsOneId.second)->asWorldRobot());
            }
        }
        for (const auto &kalmanBlueBotsOneId : blueBots) {
            if (!kalmanBlueBotsOneId.second.empty()) {
                world.mutable_blue()->Add(bestFilter(kalmanBlueBotsOneId.second)->asWorldRobot());
            }
        }

        if (!balls.empty()){
            proto::WorldBall worldBall =bestFilter(balls)->asWorldBall();
            world.mutable_ball()->CopyFrom(worldBall);
        }
        return world;
    }
    void WorldFilter::update(double time, bool extrapolateLastStep) {
        //TODO: Find a more pretty way to write a loop like this
        const double removeFilterTime=0.4; //Remove filters if no new observations have been added to it for this amount of time
        for (auto& filtersAndId : yellowBots) {
            auto filter=filtersAndId.second.begin();
            while (filter != filtersAndId.second.end()){
                filter->get()->update(time,extrapolateLastStep);
                if (time-filter->get()->getLastFrameTime()>removeFilterTime){
                    filtersAndId.second.erase(filter);
                }
                else{
                    ++filter;
                }
            }
        }
        for (auto& filtersAndId : blueBots) {
            auto filter=filtersAndId.second.begin();
            while (filter != filtersAndId.second.end()){
                filter->get()->update(time,extrapolateLastStep);
                if (time-filter->get()->getLastFrameTime()>removeFilterTime){
                    filtersAndId.second.erase(filter);
                }
                else{
                    ++filter;
                }
            }
        }
        auto ball=balls.begin();
        while (ball != balls.end()){
            ball->get()->update(time,extrapolateLastStep);
            if (time-ball->get()->getLastFrameTime()>0.4){
                balls.erase(ball);
            }
            else{
                ++ball;
            }
        }
    }
    const std::unique_ptr<RobotFilter>& WorldFilter::bestFilter(const std::vector<std::unique_ptr<RobotFilter>> &filters) {
        int bestIndex=0;
        int bestFrames=0;
        for (int i = 0; i <filters.size() ; ++i) {
            if (filters[i]->frames()>bestFrames){
                bestFrames=filters[i]->frames();
                bestIndex=i;
            }
        }
        return filters[bestIndex];
    }
    const std::unique_ptr<BallFilter> & WorldFilter::bestFilter(const std::vector<std::unique_ptr<BallFilter>> &filters) {
        int bestIndex=-1;
        int bestFrames=-1;
        for (int i = 0; i <filters.size() ; ++i) {
            if (filters[i]->frames()>bestFrames&&filters[i]->ballIsVisible()){
                bestFrames=filters[i]->frames();
                bestIndex=i;
            }
        }
        //if we haven't found anything we throw away the visibility constraint
        if (bestIndex==-1){
            for (int i = 0; i <filters.size() ; ++i) {
                if (filters[i]->frames()>bestFrames){
                    bestFrames=filters[i]->frames();
                    bestIndex=i;
                }
            }
        }
        return filters[bestIndex];
    }
}