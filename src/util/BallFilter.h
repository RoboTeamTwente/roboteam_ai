//
// Created by rolf on 17-11-19.
//

#ifndef RTT_BALLFILTER_H
#define RTT_BALLFILTER_H

#include <roboteam_proto/messages_robocup_ssl_detection.pb.h>
#include <roboteam_proto/WorldBall.pb.h>
#include "KalmanFilter.h"

class BallFilter {
    typedef KalmanFilter<4, 2> Kalman;
public:
    //TODO: add documentation
    explicit BallFilter(const proto::SSL_DetectionBall &detectionBall, double detectTime);
    void predict(double time, bool permanentUpdate);
    void update(double time, bool doLastPredict);;
    void addObservation(const proto::SSL_DetectionBall &detectionBall, double time);
    /**
     * Distance of the state of the filter to a point.
     * @param x xCoordinate (in millimeters!)
     * @param y yCoordinate (in millimeters!)
     * @return Distance from the state to the point (x,y)
     * */
    double distanceTo(double x, double y) const;
    /**
     * Outputs the current filter state in proto format.
     * @return The Proto message associated with the state of the filter
     */
    proto::WorldBall asWorldBall() const;
    /**
     * The time of the last observation which was processed by the filter
     * @return The time at which the filter was last updated
     */
    double getLastFrameTime() const;
    /**
     * The amount of observations the filter has processed
     * @return The amount of observations the filter processed
     */
    int frames() const;
    bool ballIsVisible() const;
    /**
     * A struct to keep Ball Data and time as one observation.
     */
    struct BallObservation{
        explicit BallObservation(double time,const proto::SSL_DetectionBall& detectionBall) :
                time(time),
                bot(detectionBall)
        {}
        double time;
        proto::SSL_DetectionBall bot;
    };
private:
    /**
     * Applies the observation to the kalman Filter at the current time the filter is at.
     * This changes the z and r matrices.
     * Make sure you have predicted until the correct time before calling this!
     * @param detectionBall Ball to be applied to the filter
     */
    void applyObservation(const proto::SSL_DetectionBall &detectionBall);
    /**
     * Initializes the kalman Filter structures
     * @param detectionBall Contains the initial state of the Filter.
     */
    void KalmanInit(const proto::SSL_DetectionBall &detectionBall);
    std::unique_ptr<Kalman> kalman = nullptr;
    double lastUpdateTime;
    double lastPredictTime;
    int frameCount = 0;
    std::vector<BallObservation> observations;
};


#endif //RTT_BALLFILTER_H
