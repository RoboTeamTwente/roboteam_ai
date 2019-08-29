#include "roboteam_world/ros_handler.h"
#include "roboteam_utils/constants.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "messages_robocup_ssl_wrapper_legacy.pb.h"
#include <roboteam_world/net/robocup_ssl_client.h>


namespace rtt {

    /// Initiate the world ros_handler
    void RosHandler::init(rtt::WorldBase* _world) {
      KF = new kalmanFilter;
      world = _world;
        world_pub = new roboteam_proto::Publisher("world_state");

      vision_packet = new roboteam_proto::SSL_WrapperPacket;
    }


    void RosHandler::getMessages() {
      bool ok = true;
      while(ok) {
        std::cout<< "running" << std::endl;

        if (vision_client) {
          std::cout<< "client found!" << std::endl;

          // Receive current version packets.
          while (vision_client->receive(*vision_packet)) {
            std::cout<< "got something!" << std::endl;

            // Detection frame.
            if (vision_packet->has_detection()) {
              detection_callback(vision_packet->detection());
            }
          }
        }
      }


    }


    void RosHandler::kalmanLoop() {


      constexpr int DEFAULT_VISION_PORT = 10006;
      constexpr int DEFAULT_REFEREE_PORT = 10003;

      const std::string SSL_VISION_SOURCE_IP = "224.5.23.2";
      const std::string SSL_REFEREE_SOURCE_IP= "224.5.23.1";

      const std::string LOCAL_SOURCE_IP = "127.0.0.1";

      vision_client = new RoboCupSSLClient(DEFAULT_VISION_PORT, LOCAL_SOURCE_IP);

      std::cout << "Vision  : " << LOCAL_SOURCE_IP  << ":" << DEFAULT_VISION_PORT << std::endl;

      vision_client->open(true);



        double TICKRATE=1/TIMEDIFF;
        // ros::Rate rate(TICKRATE);


        std::thread thread(&RosHandler::getMessages, this);
      std::this_thread::sleep_for(std::chrono::microseconds(10000));




      std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
          std::chrono::system_clock::now().time_since_epoch()
      );

      std::chrono::milliseconds last_call_time = ms;

      bool ok = true;
          while(ok) {

            std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
                std::chrono::system_clock::now().time_since_epoch()
            );
            std::chrono::milliseconds now_time = ms;
            if(now_time > (last_call_time + std::chrono::milliseconds(40)))
            {
              filterLock.lock();
              KF->kalmanUpdate();
              roboteam_proto::World kmWorld = KF->getWorld();
              world_pub->send(kmWorld.SerializeAsString());
              filterLock.unlock();

              last_call_time = now_time;//last time is re initialized
            }
          }



        thread.join();
    }

    void RosHandler::setKalman(bool on) {
        kalman=on;
    }


    /// Callback function for /vision_detection in ros_handler
    void RosHandler::detection_callback(roboteam_proto::SSL_DetectionFrame frame) {
        if (kalman) {
          filterLock.lock();

          KF->newFrame(frame);
          filterLock.unlock();

          return;
        }
        world->detection_callback(frame);

        if (auto * fworld = dynamic_cast<FilteredWorld*>(world)) {
            // Filtered world! Special case that shit
            if (auto worldOpt = fworld->consumeMsg()) {
                world_pub->send(worldOpt->SerializeAsString());

            }
        } else {
            // Generic world approach
            // Where you can never be sure if this message is the new one
            roboteam_proto::World world_msg = world->as_message();
          world_pub->send(world_msg.SerializeAsString());
        }
    }
}
