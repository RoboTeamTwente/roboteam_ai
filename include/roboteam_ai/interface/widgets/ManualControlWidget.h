//
// Created by Lukas Bos on 14/11/2019.
//

#ifndef RTT_MANUALCONTROLWIDGET_H
#define RTT_MANUALCONTROLWIDGET_H

#include <QWidget>
#include <thread>

#include "interface/api/Toggles.h"
#include "manual/JoystickManager.h"

namespace rtt::ai::interface {
class ManualControlWidget : public QWidget {
   public:
    ManualControlWidget(QWidget *parent);

   private:
    std::thread joyThread;
    rtt::input::JoystickManager manager;
    Toggle RobotSelector;
};
}  // namespace rtt::ai::interface
#endif  // RTT_MANUALCONTROLWIDGET_H
