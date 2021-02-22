//
// Created by Lukas Bos on 14/11/2019.
//

#ifndef RTT_MANUALCONTROLWIDGET_H
#define RTT_MANUALCONTROLWIDGET_H

#include <QWidget>
#include <thread>
#include <include/roboteam_ai/manual/JoystickManager.h>

namespace rtt::ai::interface {
class ManualControlWidget : public QWidget {
   public:
    ManualControlWidget(QWidget *parent);

   private:
    std::thread joyThread;
    rtt::input::JoystickManager manager;
};
}  // namespace rtt
#endif  // RTT_MANUALCONTROLWIDGET_H
