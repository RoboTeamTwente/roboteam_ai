//
// Created by kjhertenberg on 13-5-19.
//

#include "roboteam_proto/messages_robocup_ssl_detection.pb.h"
#include <kalman/WorldFilter.h>

namespace world {

    WorldFilter::WorldFilter() {
        ball = KalmanBall();
    }

    void WorldFilter::kalmanUpdate() {
        //Updates the Kalman gain (K)
        //Updates the State (X)
        ball.kalmanUpdateK();
        ball.kalmanUpdateX();

    }

// if we get a new frame we update our observations
    void WorldFilter::addFrame(const proto::SSL_DetectionFrame &msg) {
        std::lock_guard<std::mutex> lock(filterMutex);

        double timeCapture = msg.t_capture();
        uint cameraID = msg.camera_id();
        for (const proto::SSL_DetectionRobot &robot : msg.robots_yellow()) {
            bool addedBot=false;
            for (const auto &filter : yellowBots[robot.robot_id()]) {
                if (filter->distanceTo(robot.x(),robot.y())<0.5){
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
                if (filter->distanceTo(robot.x(),robot.y())<0.5){
                    filter->addObservation(robot,timeCapture);
                    addedBot=true;
                }
            }
            if (!addedBot){
                // We create a new filter
                blueBots[robot.robot_id()].push_back(std::make_shared<RobotFilter>(robot,timeCapture));
            }
        }
        for (const proto::SSL_DetectionBall &detBall : msg.balls()) {
            ball.kalmanUpdateZ(detBall, timeCapture, cameraID);
        }

    }

//Creates a world message with the currently observed objects in it
    proto::World WorldFilter::getWorld(double time) {
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

        proto::WorldBall worldBall = ball.as_ball_message();
        world.mutable_ball()->CopyFrom(worldBall);
        return world;
    }
    void WorldFilter::update(double time, bool extrapolateLastStep) {
        //TODO: Find a more pretty way to write a loop like this
        for (auto& filtersAndId : yellowBots) {
            auto filter=filtersAndId.second.begin();
            while (filter != filtersAndId.second.end()){
                filter->get()->update(time,extrapolateLastStep);
                if (time-filter->get()->getLastFrameTime()>0.4){
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
                if (time-filter->get()->getLastFrameTime()>0.4){
                    filtersAndId.second.erase(filter);
                }
                else{
                    ++filter;
                }
            }
        }
    }
    std::shared_ptr<RobotFilter> WorldFilter::bestFilter(std::vector<std::shared_ptr<RobotFilter>> filters) {
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
}