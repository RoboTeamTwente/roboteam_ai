//
// Created by emiel on 27-10-19.
//

#ifndef RTT_JOYSTICKSTATE_H
#define RTT_JOYSTICKSTATE_H

#include <roboteam_utils/Vector2.h>

namespace rtt {
namespace input {

class JoystickState {
   public:
    rtt::Vector2 stickLeft = {0.0, 0.0};
    rtt::Vector2 stickRight = {0.0, 0.0};
    int triggerLeft = 0;
    int triggerRight = 0;
    bool bumperRight = false;
    bool A = false;
    bool B = false;
    bool X = false;
    bool Y = false;
    bool start = false;
    bool back = false;
    bool XBOX = false;
    bool dpadLeft = false;
    bool dpadRight = false;
};

}  // namespace input
}  // namespace rtt

#endif  // RTT_JOYSTICKSTATE_H