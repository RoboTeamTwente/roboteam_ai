//
// Created by mrlukasbos on 7-5-19.
//

#ifndef ROBOTEAM_AI_VISUALIZATIONSETTINGSWIDGET_H
#define ROBOTEAM_AI_VISUALIZATIONSETTINGSWIDGET_H

#include "QLayout"
#include "widget.h"

namespace rtt::ai::interface {

class VisualizationSettingsWidget : public QWidget {
    Q_OBJECT
   public:
    explicit VisualizationSettingsWidget(Visualizer *visualizer, QWidget *parent = nullptr);
};

}  // namespace rtt::ai::interface

#endif  // ROBOTEAM_AI_VISUALIZATIONSETTINGSWIDGET_H
