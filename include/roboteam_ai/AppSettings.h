//
// Created by rolf on 07-03-21.
//

#ifndef RTT_ROBOTEAM_AI_SRC_APPSETTINGS_H_
#define RTT_ROBOTEAM_AI_SRC_APPSETTINGS_H_
#include <string>
#include <roboteam_proto/Setting.pb.h>
#include <roboteam_proto/ObserverSettings.pb.h>
#include <roboteam_proto/RobotHubSettings.pb.h>
enum class SerialMode{
  GRSIM = 0,
  SERIAL = 1
};
class AppSettings {
 public:

  [[nodiscard]] const std::string& getRefereeIp() const;
  void setRefereeIp(const std::string& refereeIp);

  [[nodiscard]] int getRefereePort() const;
  void setRefereePort(int port);

  [[nodiscard]] const std::string& getVisionIp() const;
  void setVisionIp(const std::string& vision_ip);

  [[nodiscard]] int getVisionPort() const;
  void setVisionPort(int port);

  [[nodiscard]] proto::Setting toMessage() const;
  [[nodiscard]] proto::ObserverSettings obsMessage() const;

  [[nodiscard]] proto::RobotHubSettings rhMessage() const;
 private:
  std::string referee_ip;
  int referee_port;

  std::string vision_ip;
  int vision_port;

  std::string robothub_ip;
  int robothub_port;
  SerialMode mode;
};

#endif //RTT_ROBOTEAM_AI_SRC_APPSETTINGS_H_
