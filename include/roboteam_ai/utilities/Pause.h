//
// Created by baris on 15-2-19.
//

#ifndef ROBOTEAM_AI_PAUSE_H
#define ROBOTEAM_AI_PAUSE_H

#include <mutex>
#include "roboteam_proto/RobotCommand.pb.h"
#include "world/World.h"

namespace rtt::ai {

namespace io {
class IOManager;
}

class Pause {
 private:
  static bool pause;
  static std::mutex pauseLock;

 public:
  Pause();
  bool getPause();
  void haltRobots();
  void setPause(bool set);
};
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_PAUSE_H
