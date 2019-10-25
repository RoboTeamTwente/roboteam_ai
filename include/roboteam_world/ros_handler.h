#pragma once

#include "roboteam_proto/messages_robocup_ssl_detection.pb.h"
#include "roboteam_proto/World.pb.h"
#include "world/world_dummy.h"
#include "world/filtered_world.h"
#include "roboteam_proto/Subscriber.h"
#include <roboteam_proto/Publisher.h>
#include <net/robocup_ssl_client.h>
#include "kalman/kalmanFilter.h"

namespace rtt {

    class RosHandler {

    private:
        roboteam_proto::Publisher<roboteam_proto::World> * world_pub;
        roboteam_proto::Publisher<roboteam_proto::SSL_Referee> * ref_pub;
        roboteam_proto::Publisher<roboteam_proto::SSL_GeometryData> * geom_pub;

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
