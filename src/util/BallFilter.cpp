//
// Created by rolf on 17-11-19.
//

#include "BallFilter.h"
#include "Scaling.h"

BallFilter::BallFilter(const proto::SSL_DetectionBall &detectionBall, double detectTime) :
        lastUpdateTime{detectTime},
        lastPredictTime{detectTime},
        frameCount{1} {
    KalmanInit(detectionBall);
}
void BallFilter::KalmanInit(const proto::SSL_DetectionBall &detectionBall) {
    // SSL units are in mm, we do everything in SI units.
    double x = mmToM(detectionBall.x());//m
    double y = mmToM(detectionBall.y());//m
    Kalman::Vector startState = {
            x, y, 0, 0
    };

    Kalman::Matrix startCov;
    startCov.eye();
    //initial noise estimates
    const double startPosNoise = 0.3;
    const double startAngleNoise = 0.05;
    startCov.at(0, 0) = startPosNoise;//m noise in x
    startCov.at(1, 1) = startPosNoise;//m noise in y

    kalman = std::make_unique<Kalman>(startState, startCov);

    kalman->H.eye();     // Our observations are simply what we see.

    const double posVar = 0.02; //variance in meters
    kalman->R.zeros();
    kalman->R.at(0, 0) = posVar;
    kalman->R.at(1, 1) = posVar;
}
void BallFilter::applyObservation(const proto::SSL_DetectionBall &detectionBall) {
    Kalman::VectorO observation;
    observation.zeros();
    observation.at(0) = mmToM(detectionBall.x());
    observation.at(1) = mmToM(detectionBall.y());

    //TODO: do things with the other ball fields (pixel pos, area)
    kalman->z = observation;
    kalman->update();
}
int BallFilter::frames() const {
    return frameCount;
}
double BallFilter::getLastFrameTime() const {
    return lastUpdateTime;
}
proto::WorldBall BallFilter::asWorldBall() const {
    proto::WorldBall msg;
    const Kalman::Vector &state = kalman->state();
    msg.mutable_pos()->set_x(state[0]);
    msg.mutable_pos()->set_y(state[1]);
    msg.mutable_vel()->set_x(state[2]);
    msg.mutable_vel()->set_y(state[3]);
    msg.set_visible(ballIsVisible());
    //TODO: add height filter here, and actually set the z, z_vel, area fields
    return msg;
}
double BallFilter::distanceTo(double x, double y) const {
    const Kalman::Vector &state = kalman->state();
    double dx = state[0] - mmToM(x);
    double dy = state[1] - mmToM(y);
    return sqrt(dx * dx + dy * dy);
}
void BallFilter::predict(double time, bool permanentUpdate) {
    double dt = time - lastUpdateTime;
    // forward model:
    kalman->F.eye();
    kalman->F.at(0, 2) = dt;
    kalman->F.at(1, 3) = dt;

    //TODO: add 2 stage forward model?
    //Set B
    kalman->B = kalman->F;
    //Set u (we have no control input at the moment)
    kalman->u.zeros();

    //Set Q matrix
    Kalman::MatrixO G;
    G.zeros();
    G.at(0, 0) = dt;
    G.at(0, 2) = 1;
    G.at(1, 1) = dt;
    G.at(1, 3) = 1;

    const float processNoise = 0.01;
    kalman->Q = G.t() * G * processNoise;

    kalman->predict(permanentUpdate);
    lastPredictTime=time;
    if (permanentUpdate) {
        lastUpdateTime = time;
    }
}
bool compareObservation(const BallFilter::BallObservation& a, const BallFilter::BallObservation& b ){
    return (a.time<b.time);
}
void BallFilter::update(double time, bool doLastPredict) {

    std::sort(observations.begin(),observations.end(),compareObservation); //First sort the observations in time increasing order
    auto it=observations.begin();
    while(it != observations.end()) {
        auto observation=(*it);
        //the observation is either too old (we already updated the robot) or too new and we don't need it yet.
        if (observation.time<lastUpdateTime) {
            observations.erase(it);
            continue;
        }
        if(observation.time>time){
            //relevant update, but we don't need the info yet so we skip it.
            ++it;
            continue;
        }
        // We first predict the ball, and then apply the observation to calculate errors/offsets.
        predict(observation.time,true);
        applyObservation(observation.bot);
        observations.erase(it);
    }
    if(doLastPredict){
        predict(time,false);
    }

}
void BallFilter::addObservation(const proto::SSL_DetectionBall &detectionBall, double time) {
    observations.emplace_back(BallObservation(time, detectionBall));
    frameCount++;
}
bool BallFilter::ballIsVisible() const {
    //If we extrapolated the ball for longer than 0.05 seconds we mark it not visible
    return (lastPredictTime-lastUpdateTime)<0.05;
}
