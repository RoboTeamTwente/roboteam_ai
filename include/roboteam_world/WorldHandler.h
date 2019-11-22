#ifndef WORLDHANDLER_H
#define WORLDHANDLER_H
#include "roboteam_proto/messages_robocup_ssl_detection.pb.h"
#include "roboteam_proto/World.pb.h"
#include "roboteam_proto/Subscriber.h"
#include <roboteam_proto/Publisher.h>
#include <net/robocup_ssl_client.h>
#include "kalman/WorldFilter.h"
#include "util/KalmanFilter.h"

namespace world {

class WorldHandler {
 private:
  proto::Publisher<proto::World> *world_pub;
  proto::Publisher<proto::SSL_Referee> *ref_pub;
  proto::Publisher<proto::SSL_GeometryData> *geom_pub;

  double lastPacketTime;
  WorldFilter *KF;
  RoboCupSSLClient *vision_client;
  RoboCupSSLClient *refbox_client;

 public:
  WorldHandler() = default;

  /*
   * Setup a world with a kalmanfilter, and initialize the publishers for publishing data.
   */
  void init();
  void start();
  void handleVisionPackets(proto::SSL_WrapperPacket &vision_packet);
  void handleRefboxPackets(proto::SSL_Referee &ref_packet) const;
  void setupSSLClients();
};

}
#endif