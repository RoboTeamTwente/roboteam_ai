//
// Created by mrlukasbos on 7-5-19.
//

#ifndef ROBOTEAM_AI_VISUALIZATIONSETTINGSWIDGET_H
#define ROBOTEAM_AI_VISUALIZATIONSETTINGSWIDGET_H


#include "widget.h"

namespace rtt {
namespace ai {
namespace interface {

class VisualizationSettingsWidget : public QWidget {
Q_OBJECT
public:
    explicit VisualizationSettingsWidget(Visualizer * visualizer, QWidget * parent = nullptr);
};

} // interface
} // ai
} // rtt



#endif //ROBOTEAM_AI_VISUALIZATIONSETTINGSWIDGET_H
