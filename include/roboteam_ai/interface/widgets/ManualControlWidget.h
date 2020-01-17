//
// Created by Lukas Bos on 14/11/2019.
//

#ifndef RTT_MANUALCONTROLWIDGET_H
#define RTT_MANUALCONTROLWIDGET_H

#include <QWidget>
#include <thread>
#include <include/roboteam_ai/interface/api/Toggles.h>

namespace rtt {
namespace input {
class JoystickManager;
}
namespace ai {
namespace interface {
class ManualControlWidget : public QWidget {
   public:
    ManualControlWidget(QWidget* parent);

   private:
    std::thread joyThread;
    rtt::input::JoystickManager* manager;
    Toggle RobotSelector;
};
}  // namespace interface
}  // namespace ai
}  // namespace rtt
#endif  // RTT_MANUALCONTROLWIDGET_H
