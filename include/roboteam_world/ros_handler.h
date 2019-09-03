#pragma once

#include "roboteam_proto/messages_robocup_ssl_detection.pb.h"
#include "roboteam_proto/World.pb.h"
#include "roboteam_world/world/world_dummy.h"
#include "roboteam_world/world/filtered_world.h"
#include "roboteam_proto/Subscriber.h"
#include <roboteam_proto/Publisher.h>
#include <roboteam_world/net/robocup_ssl_client.h>
#include "kalman/kalmanFilter.h"

namespace rtt {

    class RosHandler {

    private:
        roboteam_proto::Publisher * pub;

      WorldBase* world;
        bool kalman;
        RoboCupSSLClient * vision_client;
        RoboCupSSLClient * refbox_client;
     public:
        RosHandler() = default;
        void init(WorldBase* _world);
        void kalmanLoop();
        void setKalman(bool on);
        void detection_callback(roboteam_proto::SSL_DetectionFrame frame);
    //    bool reset_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        kalmanFilter * KF;
        std::mutex filterLock;
    };

}
