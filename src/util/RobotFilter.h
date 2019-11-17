//
// Created by rolf on 05-11-19.
//

#ifndef RTT_ROBOTFILTER_H
#define RTT_ROBOTFILTER_H


#include <roboteam_proto/messages_robocup_ssl_detection.pb.h>
#include <roboteam_proto/WorldRobot.pb.h>
#include "KalmanFilter.h"

/**
 * A class that can filter robots and predict where they will be based on observations.
 * @author Rolf
 * @date 5 November 2019
 */
class RobotFilter {
    typedef KalmanFilter<6,3> Kalman;
public:
    /**
     * Construct a RobotFilter.
     * @param detectionRobot Initial observation of the robot we start our filter with.
     * @param detectTime Point in time we start the filter at.
     */
    explicit RobotFilter(const proto::SSL_DetectionRobot& detectionRobot, double detectTime);
    /**
     * Predicts the state of the robot based on past observations
     * @param time The time at which we wish to have a prediction of where the robot will be
     * @param permanentUpdate If set to true, the update is applied permanently to the filter.
     * If not, we may still add new observations from after the last time the Filter was between the variable time
     * and the last time the filter was permanently updated.
     */
    void predict(double time, bool permanentUpdate);
    /**
     * Updates the Filter until the specified time, applying observations of the robot and predicting the state along the way.
     * @param time Time until which we want to update.
     * @param doLastPredict In the very last step after applying all the observations, we can choose to not do the last
     * prediction if we do not immediately want to read the filter's data.
     */
    void update(double time, bool doLastPredict);
    /**
     * Adds an observation of the robot to the filter.
     * @param detectionRobot State of the robot that was observed
     * @param time Time the robot was observed
     */
    void addObservation(const proto::SSL_DetectionRobot& detectionRobot, double time);
    /**
     * Distance of the state of the filter to a point.
     * @param x xCoordinate (in millimeters!)
     * @param y yCoordinate (in millimeters!)
     * @return Distance from the state to the point (x,y)
     */
    double distanceTo(double x, double y) const;
    /**
     * Outputs the current filter state in proto format.
     * @return The Proto message associated with the state of the filter
     */
    proto::WorldRobot asWorldRobot() const;
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
    /**
     * A struct to keep robotData and time as one observation.
     */
    struct RobotObservation{
        explicit RobotObservation(double time,const proto::SSL_DetectionRobot& detectionRobot) :
        time(time),
        bot(detectionRobot)
        {}
        double time;
        proto::SSL_DetectionRobot bot;
    };
private:
    /**
     * Applies the observation to the kalman Filter at the current time the filter is at.
     * This changes the z and r matrices.
     * Make sure you have predicted until the correct time before calling this!
     * @param detectionRobot Robot to be applied
     */
    void applyObservation(const proto::SSL_DetectionRobot& detectionRobot);
    /**
     * A function that casts any angle to the range [-PI,PI)
     * @param angle angle to be limited
     * @return Limited angle in range [-PI,PI)
     */
    double limitAngle(double angle) const;
    /**
     * Initializes the kalman Filter structures
     * @param detectionRobot Contains the initial state of the Filter.
     */
    void KalmanInit(const proto::SSL_DetectionRobot& detectionRobot);
    std::unique_ptr<Kalman> kalman= nullptr;
    double lastUpdateTime;
    int frameCount=0;
    int botId;
    std::vector<RobotObservation> observations;

};


#endif //RTT_ROBOTFILTER_H
