//
// Created by thijs on 17-6-19.
//

#ifndef ROBOTEAM_AI_CHECKBOXWIDGET_H
#define ROBOTEAM_AI_CHECKBOXWIDGET_H

#include "QLayout"
#include "widget.h"

namespace rtt {
namespace ai {
namespace interface {

enum CheckboxType {
  visualize,
  manualRobots
};

class CheckboxWidget : public QWidget {
    Q_OBJECT
    public:
        explicit CheckboxWidget(Visualizer* visualizer, QWidget* parent = nullptr,
                CheckboxType checkboxType = CheckboxType::visualize);

    private:
        void visualizeCheckbox(Visualizer* visualizer, QWidget* parent);
        void manualRobotsCheckbox(Visualizer* visualizer, QWidget* parent);

};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_CHECKBOXWIDGET_H
