//
// Created by rolf on 05-11-19.
//

#ifndef RTT_ROBOTFILTER_H
#define RTT_ROBOTFILTER_H


#include <roboteam_proto/messages_robocup_ssl_detection.pb.h>
#include <roboteam_proto/WorldRobot.pb.h>
#include "KalmanFilter.h"

class RobotFilter {
    typedef KalmanFilter<6,3> Kalman;
public:
    explicit RobotFilter(const proto::SSL_DetectionRobot& detectionRobot, double detectTime);
    void predict(double time, bool permanentUpdate);
    void update(double time, bool doLastPredict);
    void addObservation(const proto::SSL_DetectionRobot& detectionRobot, double time);
    double distanceTo(double x, double y) const;
    proto::WorldRobot asWorldRobot() const;
    int frames() const;
    struct RobotObservation{
        explicit RobotObservation(double time,const proto::SSL_DetectionRobot& detectionRobot) :
        time(time),
        bot(detectionRobot)
        {}
        double time;
        proto::SSL_DetectionRobot bot;
    };
private:
    void applyObservation(const proto::SSL_DetectionRobot& detectionRobot);
    double limitAngle(double angle) const;
    void KalmanInit(const proto::SSL_DetectionRobot& detectionRobot);
    std::unique_ptr<Kalman> kalman= nullptr;
    double lastUpdateTime;
    int frameCount=0;
    int botId;
    std::vector<RobotObservation> observations;

};


#endif //RTT_ROBOTFILTER_H
