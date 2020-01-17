//
// Created by Lukas Bos on 14/11/2019.
//

#ifndef RTT_JOYSTICKMANAGER_H
#define RTT_JOYSTICKMANAGER_H


#include <memory>
#include <map>
#include <unistd.h>
#include <mutex>
#include <SDL2/SDL.h>
#include <SDL2/SDL_joystick.h>
#include <include/roboteam_ai/io/IOManager.h>
#include "JoystickHandler.h"

namespace rtt {
namespace input {

class JoystickManager {
 public:
  JoystickManager(ai::io::IOManager * ioManager);
  bool run();
  void activate();
  void deactivate();
  void stop();

 private:
  ai::io::IOManager * ioManager = nullptr;
  const int TICK_INTERVAL = 20;
  std::map<int, JoystickHandler*> joystickHandlers;

  // TODO Check if these can be non-static
  static std::mutex runningLock;
  static std::mutex activeLock;
  // Indicates whether the loop should stop
  bool running = false;
  // Indicates whether packets should be handled and joystickHandlers ticked
  bool active = false;

  bool init();
  void loop();
  bool isRunning();
  bool isActive();
  void handleJoystickAdded(const SDL_Event& event);
  void handleJoystickRemoved(const SDL_Event& event);
  void tickJoystickHandlers();
  void handleEvent(SDL_Event& event);
};

}
}



#endif //RTT_JOYSTICKMANAGER_H
